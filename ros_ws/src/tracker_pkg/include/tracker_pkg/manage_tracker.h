#pragma once

#include "tracker_pkg/global_parameters.h"
#include "tracker_pkg/kalmanfilter.h"
#include "tracker_pkg/hungarian.h"

//make namespace.
using Track2DEntry = std::tuple<double, int, cv::Rect2d>;   // time, label, bbox
using Track2DSeq = std::vector<Track2DEntry>;

struct TrackerState {
	KalmanFilter2D _kf;
	double time_last_update;//time to update info last [s]
	unsigned int label;//object label for multi-class-multi-object tracker.
	unsigned int counter_update;//number of update.
};

class TrackerManager {
private:
	HungarianAlgorithm _hungarian;
	const unsigned int _counter_update_valid = 10;//10 times update to make the tracker valid one.
	const double _cost_max = 1e5;// 10.0;
	const double _time_lost = 2.0;//2.0 second missing -> loss.
public:
	const unsigned int _n_minimum_valid_sequence = 10;//minimum valid sequence.

	std::vector<TrackerState> state_trackers;//contain tracker information internally.
	std::vector<Track2DSeq> saved_data;
	std::vector<Track2DSeq> storage_2d_kf;
	std::vector<Track2DSeq> saved_2d_kf;


	TrackerManager(){}//Constructor

	std::vector<int> update2D(
		const double& time_current,
		std::vector<cv::Rect2d>& new_trackers,
		std::vector<int>& labels,
		std::vector<Track2DSeq>& storage_2d
	);

	double calculate_iou(cv::Rect2d& bbox1,cv::Rect2d& bbox2);

	double calculate_distance_size_cost(const cv::Rect2d& bbox1, const cv::Rect2d& bbox2);

};