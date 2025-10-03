#ifndef STATS_H
#define STATS_H

struct Stats {
  int matches;
  int inliers;
  double ratio;
  long int time;
  int keypoints;

  Stats() : matches(0), inliers(0), ratio(0), time(0), keypoints(0) {}

  Stats& operator+=(const Stats& op) {
    matches += op.matches;
    inliers += op.inliers;
    ratio += op.ratio;
    time += op.time;
    keypoints += op.keypoints;
    return *this;
  }
  Stats& operator/=(int num) {
    matches /= num;
    inliers /= num;
    ratio /= num;
    time /= num;
    keypoints /= num;
    return *this;
  }
};

#endif  // STATS_H
