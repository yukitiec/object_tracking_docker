#include "tracker_pkg/manage_tracker.h"

std::vector<int> TrackerManager::update2D(
    const double& time_current, //[s]
    std::vector<cv::Rect2d>& new_trackers,
    std::vector<int>& labels,
    std::vector<Track2DSeq>& storage_2d
)
{
    // helper function
    auto stateToRect2d = [](const cv::Mat& state, double width, double height) -> cv::Rect2d
        {
            double cx = state.at<double>(0, 0);
            double cy = state.at<double>(1, 0);

            return cv::Rect2d(
                cx - width / 2.0,
                cy - height / 2.0,
                width,
                height
            );
        };

    // prepare estimated boxes from existing trackers
    std::vector<cv::Rect2d> estimated_bbox;
    std::vector<int> estimated_labels;

    for (size_t i = 0; i < storage_2d.size(); i++) {
        const Track2DEntry& d = storage_2d[i].back();
        double t_last = std::get<0>(d);
        int label_last = std::get<1>(d);
        cv::Rect2d bbox_last = std::get<2>(d);

        (void)t_last; // not used here

        estimated_labels.push_back(label_last);

        unsigned int counter_update = state_trackers[i].counter_update;
        if (counter_update >= _counter_update_valid) {
            cv::Mat pred_kf;
            state_trackers[i]._kf.predict_only(pred_kf, time_current);

            estimated_bbox.push_back(
                stateToRect2d(pred_kf, bbox_last.width, bbox_last.height)
            );
        }
        else {
            estimated_bbox.push_back(bbox_last);
        }
    }

    if (!new_trackers.empty() && !estimated_bbox.empty()) {
        std::vector<std::vector<double>> cost_matrix;

        for (int i_new = 0; i_new < static_cast<int>(new_trackers.size()); i_new++) {
            std::vector<double> cost_row;
            cv::Rect2d bbox_new = new_trackers[i_new];
            int label_new = labels[i_new];

            for (int i_old = 0; i_old < static_cast<int>(estimated_bbox.size()); i_old++) {
                cv::Rect2d bbox_old = estimated_bbox[i_old];
                int label_old = estimated_labels[i_old];

                double cost_total = calculate_distance_size_cost(bbox_new, bbox_old);

                if (label_new != label_old) {
                    cost_total += _cost_max;
                }

                cost_row.push_back(cost_total);
            }
            cost_matrix.push_back(cost_row);
        }

        // Hungarian algorithm
        std::vector<int> assignment;
        std::vector<int> idx_associated_trackers;
        std::vector<std::pair<int, int>> matching;

        double cost = _hungarian.Solve(cost_matrix, assignment);
        (void)cost;

        for (int i_new = 0; i_new < static_cast<int>(assignment.size()); i_new++) {
            int i_old = assignment[i_new];
            if (i_old >= 0) {
                if (cost_matrix[i_new][i_old] < _cost_max) {
                    matching.push_back({ i_old, i_new });
                }
            }
        }

        // Update matched trackers
        if (!matching.empty()) {
            for (std::pair<int, int>& pair_match : matching) {
                int i_old = pair_match.first;
                int i_new = pair_match.second;
                idx_associated_trackers.push_back(i_new);

                cv::Rect2d bbox_new = new_trackers[i_new];
                int label_new = labels[i_new];

                // Save raw detection
                storage_2d[i_old].push_back(
                    std::make_tuple(time_current, label_new, bbox_new)
                );

                // Center measurement
                cv::Mat measurement = cv::Mat::zeros(2, 1, CV_64F);
                measurement.at<double>(0, 0) = bbox_new.x + bbox_new.width / 2.0;
                measurement.at<double>(1, 0) = bbox_new.y + bbox_new.height / 2.0;

                state_trackers[i_old]._kf.predict(time_current);
                state_trackers[i_old]._kf.update(measurement);

                // Save KF bbox with same width/height
                cv::Mat state_kf = state_trackers[i_old]._kf.getState();
                cv::Rect2d bbox_kf = stateToRect2d(state_kf, bbox_new.width, bbox_new.height);

                storage_2d_kf[i_old].push_back(
                    std::make_tuple(time_current, label_new, bbox_kf)
                );

                // keep tracker's current label in internal state too
                state_trackers[i_old].label = label_new;
                state_trackers[i_old].time_last_update = time_current;
            }

            // Add unmatched new detections as new trackers
            if (idx_associated_trackers.size() < new_trackers.size()) {
                std::sort(idx_associated_trackers.rbegin(), idx_associated_trackers.rend());

                for (const int i_del : idx_associated_trackers) {
                    if (i_del >= 0 && i_del < static_cast<int>(new_trackers.size())) {
                        new_trackers.erase(new_trackers.begin() + i_del);
                        labels.erase(labels.begin() + i_del);
                    }
                }

                for (int i_new = 0; i_new < static_cast<int>(new_trackers.size()); ++i_new) {
                    cv::Rect2d bbox_new = new_trackers[i_new];
                    int label_new = labels[i_new];

                    // raw storage
                    storage_2d.push_back({
                        std::make_tuple(time_current, label_new, bbox_new)
                        });

                    // measurement = center
                    cv::Mat measurement = cv::Mat::zeros(2, 1, CV_64F);
                    measurement.at<double>(0, 0) = bbox_new.x + bbox_new.width / 2.0;
                    measurement.at<double>(1, 0) = bbox_new.y + bbox_new.height / 2.0;

                    KalmanFilter2D kf_inst(
                        bbox_new.x + bbox_new.width / 2.0,
                        bbox_new.y + bbox_new.height / 2.0,
                        0.0, 0.0,
                        0.0, 0.0,
                        NOISE_POS, NOISE_VEL, NOISE_ACC, NOISE_SENSOR
                    );

                    kf_inst.predict(time_current);
                    kf_inst.update(measurement);

                    TrackerState tmp{
                        kf_inst,
                        time_current,
                        label_new,
                        1
                    };

                    state_trackers.push_back(tmp);

                    // KF storage
                    cv::Mat state_kf = kf_inst.getState();
                    cv::Rect2d bbox_kf = stateToRect2d(state_kf, bbox_new.width, bbox_new.height);

                    storage_2d_kf.push_back({
                        std::make_tuple(time_current, label_new, bbox_kf)
                        });
                }
            }
        }
        else {
            // No matching at all -> all new detections become new trackers
            for (int i_new = 0; i_new < static_cast<int>(new_trackers.size()); ++i_new) {
                cv::Rect2d bbox_new = new_trackers[i_new];
                int label_new = labels[i_new];

                storage_2d.push_back({
                    std::make_tuple(time_current, label_new, bbox_new)
                    });

                cv::Mat measurement = cv::Mat::zeros(2, 1, CV_64F);
                measurement.at<double>(0, 0) = bbox_new.x + bbox_new.width / 2.0;
                measurement.at<double>(1, 0) = bbox_new.y + bbox_new.height / 2.0;

                KalmanFilter2D kf_inst(
                    bbox_new.x + bbox_new.width / 2.0,
                    bbox_new.y + bbox_new.height / 2.0,
                    0.0, 0.0,
                    0.0, 0.0,
                    NOISE_POS, NOISE_VEL, NOISE_ACC, NOISE_SENSOR
                );

                kf_inst.predict(time_current);
                kf_inst.update(measurement);

                TrackerState tmp{
                    kf_inst,
                    time_current,
                    label_new,
                    1
                };

                state_trackers.push_back(tmp);

                cv::Mat state_kf = kf_inst.getState();
                cv::Rect2d bbox_kf = stateToRect2d(state_kf, bbox_new.width, bbox_new.height);

                storage_2d_kf.push_back({
                    std::make_tuple(time_current, label_new, bbox_kf)
                    });
            }
        }
    }
    else if (!new_trackers.empty() && estimated_bbox.empty()) {
        // First update -> initialize storage and kalman filter
        for (int i_new = 0; i_new < static_cast<int>(new_trackers.size()); i_new++) {
            cv::Rect2d bbox_new = new_trackers[i_new];
            int label_new = labels[i_new];

            storage_2d.push_back({
                std::make_tuple(time_current, label_new, bbox_new)
                });

            cv::Mat measurement = cv::Mat::zeros(2, 1, CV_64F);
            measurement.at<double>(0, 0) = bbox_new.x + bbox_new.width / 2.0;
            measurement.at<double>(1, 0) = bbox_new.y + bbox_new.height / 2.0;

            KalmanFilter2D kf_inst(
                bbox_new.x + bbox_new.width / 2.0,
                bbox_new.y + bbox_new.height / 2.0,
                0.0, 0.0,
                0.0, 0.0,
                NOISE_POS, NOISE_VEL, NOISE_ACC, NOISE_SENSOR
            );

            kf_inst.predict(time_current);
            kf_inst.update(measurement);

            TrackerState tmp{
                kf_inst,
                time_current,
                label_new,
                1
            };

            state_trackers.push_back(tmp);

            cv::Mat state_kf = kf_inst.getState();
            cv::Rect2d bbox_kf = stateToRect2d(state_kf, bbox_new.width, bbox_new.height);

            storage_2d_kf.push_back({
                std::make_tuple(time_current, label_new, bbox_kf)
                });
        }
    }

    // Delete lost trackers
    std::vector<int> index_delete_storage;
    for (int i_obj = 0; i_obj < static_cast<int>(storage_2d.size()); i_obj++) {
        const Track2DEntry& d_last = storage_2d[i_obj].back();
        double dt = time_current - std::get<0>(d_last);

        if (dt > _time_lost) {
            if (storage_2d[i_obj].size() >= _n_minimum_valid_sequence) {
                saved_data.push_back(storage_2d[i_obj]);
                saved_2d_kf.push_back(storage_2d_kf[i_obj]);
            }
            index_delete_storage.push_back(i_obj);
        }
    }

    if (!index_delete_storage.empty()) {
        std::sort(index_delete_storage.rbegin(), index_delete_storage.rend());
        for (int& i_del : index_delete_storage) {
            storage_2d.erase(storage_2d.begin() + i_del);
            storage_2d_kf.erase(storage_2d_kf.begin() + i_del);
            state_trackers.erase(state_trackers.begin() + i_del);
        }
    }

    return index_delete_storage;
}

double TrackerManager::calculate_iou(cv::Rect2d& bbox1, cv::Rect2d& bbox2) {
    // Find intersection rectangle
    double x_left = std::max(bbox1.x, bbox2.x);
    double y_top = std::max(bbox1.y, bbox2.y);
    double x_right = std::min(bbox1.x + bbox1.width, bbox2.x + bbox2.width);
    double y_bottom = std::min(bbox1.y + bbox1.height, bbox2.y + bbox2.height);

    // Compute intersection area
    double intersection_width = std::max(0.0, x_right - x_left);
    double intersection_height = std::max(0.0, y_bottom - y_top);
    double intersection_area = intersection_width * intersection_height;

    // Compute union area
    double area1 = bbox1.width * bbox1.height;
    double area2 = bbox2.width * bbox2.height;
    double union_area = area1 + area2 - intersection_area;

    // Avoid division by zero
    if (union_area <= 0.0)
        return 0.0;

    return intersection_area / union_area;
}

double TrackerManager::calculate_distance_size_cost(const cv::Rect2d& bbox1, const cv::Rect2d& bbox2) {
    // Compute centers of bbox1 and bbox2
    double cx1 = bbox1.x + bbox1.width / 2.0;
    double cy1 = bbox1.y + bbox1.height / 2.0;
    double cx2 = bbox2.x + bbox2.width / 2.0;
    double cy2 = bbox2.y + bbox2.height / 2.0;

    // 2D Euclidean distance between centers
    double distance = std::sqrt((cx1 - cx2) * (cx1 - cx2) + (cy1 - cy2) * (cy1 - cy2));

    // Absolute size (area) difference
    double area1 = bbox1.width * bbox1.height;
    double area2 = bbox2.width * bbox2.height;
    double size_diff = std::abs(area1 - area2);

    // Sum of distance cost and size difference cost
    return distance + size_diff;
}