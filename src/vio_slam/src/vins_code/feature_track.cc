#include "feature_track.hh"

namespace vslam::vins {

bool FeatureTrack::inBorder(const cv::Point2f &point) {
    const int BORDER_SIZE = 1;
    int img_x = cvRound(point.x);
    int img_y = cvRound(point.y);
    return BORDER_SIZE <= img_x && img_x < config_.COL - BORDER_SIZE && BORDER_SIZE <= img_y &&
           img_y < config_.ROW - BORDER_SIZE;
    return 0;
}
FeatureTrack::FeatureTrack(const SystemConfg &config) : config_(config) {
}

FeatureTrack::~FeatureTrack() {
}

void FeatureTrack::ReadIntrinsicParameter(const std::string &config_file) {
    std::cout << "read parameter of camera" << config_file << std::endl;
    camera_ptr = camodocal::CameraFactory::instance()->generateCameraFromYamlFile(config_file);
}

// template <typename T>
// void FeatureTrack::reduceVector(T &&datas, std::vector<uchar> status) {
//     int valid_index = 0;
//     for (int i = 0; i < datas.size(); ++i) {
//         if (status[i]) {
//             datas[valid_index++] = datas[i];
//         }
//     }
//     datas.resize(valid_index);
// }

// 对已经提取的关键点进行了mask
void FeatureTrack::setMask() {
    // mask是全黑的，后面对已经追踪的关键点绘制成白色
    mask = cv::Mat(config_.ROW, config_.COL, CV_8UC1, cv::Scalar(255));
    if (forw_img.empty()) {
        return;
    }
    // track_cnt, feature_point2f, feature_id
    std::vector<std::pair<int, std::pair<cv::Point2f, int>>> cnt_pts_ids;
    for (unsigned int i = 0; i < forw_kps.size(); ++i) {
        cnt_pts_ids.push_back(std::make_pair(track_cnt[i], std::make_pair(forw_kps[i], ids[i])));
    }
    // sort
    sort(cnt_pts_ids.begin(), cnt_pts_ids.end(), [](const auto &a, const auto &b) { return a.first > b.first; });

    forw_kps.clear();
    ids.clear();
    track_cnt.clear();

    for (const auto &it : cnt_pts_ids) {
        if (mask.at<uchar>(it.second.first) == 255) {
            forw_kps.push_back(it.second.first);
            ids.push_back(it.second.second);
            track_cnt.push_back(it.first);
            cv::circle(mask, it.second.first, config_.MIN_DIST, 0, -1);
        }
    }
}

// 对光流追踪到的点去畸变之后，进行基础矩阵的估计
void FeatureTrack::rejectWithF() {
    if (forw_kps.size() > 8) {
        std::vector<cv::Point2f> cur_undistor_pts(cur_kps.size());
        std::vector<cv::Point2f> forw_undistor_pts(forw_kps.size());
        assert(cur_undistor_pts.size() == forw_undistor_pts.size());
        for (size_t i = 0; i < cur_undistor_pts.size(); ++i) {
            Eigen::Vector3d tmp_pt;
            camera_ptr->liftProjective(Eigen::Vector2d(cur_kps[i].x, cur_kps[i].y), tmp_pt);
            // 相机点到像素点
            tmp_pt.x() = config_.focal_length * tmp_pt.x() / tmp_pt.z() + config_.ROW / 2;
            tmp_pt.y() = config_.focal_length * tmp_pt.y() / tmp_pt.z() + config_.COL / 2;
            cur_undistor_pts[i] = cv::Point2f(tmp_pt.x(), tmp_pt.y());

            camera_ptr->liftProjective(Eigen::Vector2d(forw_kps[i].x, forw_kps[i].y), tmp_pt);
            tmp_pt.x() = config_.focal_length * tmp_pt.x() / tmp_pt.z() + config_.ROW / 2;
            tmp_pt.y() = config_.focal_length * tmp_pt.y() / tmp_pt.z() + config_.COL / 2;
            forw_undistor_pts[i] = cv::Point2f(tmp_pt.x(), tmp_pt.y());
        }

        std::vector<uchar> status;
        cv::findFundamentalMat(cur_undistor_pts, forw_undistor_pts, cv::FM_RANSAC, config_.F_THRESHOLD, 0.99, status);
        reduceVector(prev_kps, status);
        reduceVector(cur_kps, status);
        reduceVector(forw_kps, status);
        reduceVector(ids, status);
        reduceVector(cur_un_kps, status);
        reduceVector(track_cnt, status);
    }
}

void FeatureTrack::addPoints() {
    for (const auto &pt : kpts) {
        forw_kps.push_back(pt);
        ids.push_back(-1);
        track_cnt.push_back(1);
    }
}

void FeatureTrack::trackImage(const cv::Mat &image, double img_time, bool track_this_frame) {
    cv::Mat img;
    cur_time = img_time;

    if (config_.EQUALIZE) {
        cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(3.0, cv::Size(8, 8));
        clahe->apply(image, img);
    } else {
        img = image;
    }
    // 如果是第一帧的情况下
    if (!forw_img.empty()) {
        prev_img = cur_img = forw_img = img;
    } else {
        // 正在处理的image
        forw_img = img;
    }
    forw_kps.clear();
    // 上一帧的关键点
    if (!cur_kps.empty()) {
        // 光流追踪, 这里就会丢掉或者保留长期追踪下来的点。这里是每帧都在处理
        std::vector<uchar> status;
        std::vector<float> errors;
        cv::calcOpticalFlowPyrLK(cur_img, forw_img, cur_kps, forw_kps, status, errors, cv::Size(21, 21), 3);
        // 根据状态过滤掉outliner
        for (int i = 0; i < forw_kps.size(); ++i) {
            // 如果在image size外面了，需要排除改点
            if (status[i] && !inBorder(forw_kps[i])) {
                status[i] = 0;
            }
        }
        // 根据status的状态缩减各个vector;
        reduceVector(prev_kps, status);
        reduceVector(cur_kps, status);
        reduceVector(forw_kps, status);
        reduceVector(ids, status);
        reduceVector(cur_un_kps, status);
        reduceVector(track_cnt, status);
    }
    if (!track_cnt.empty()) {
        for (auto &count : track_cnt) {
            // 对追踪到的点计数累加
            count++;
        }
    }
    std::cout << "forw_kps:" << forw_kps.size() << std::endl;

    // 才开始追踪特征
    if (track_this_frame) {
        rejectWithF();
        setMask();
        // 总的特征点 - 上一帧光流追踪到的特征点个数 = 将要提取的特征点数
        std::cout << "forw_kps:" << forw_kps.size() << std::endl;
        int frame_max_features = config_.MAX_CNT - static_cast<int>(forw_kps.size());
        if (frame_max_features > 0) {
            if (mask.size() != forw_img.size()) {
                std::cerr << "wrong size" << std::endl;
            }
            cv::goodFeaturesToTrack(forw_img, kpts, config_.MAX_CNT - forw_kps.size(), 0.01, config_.MIN_DIST, mask);
        } else {
            kpts.clear();
        }
        addPoints();
        std::cout << "forw_kps:" << forw_kps.size() << std::endl;
    }
    // 状态
    prev_img = cur_img;
    prev_kps = cur_kps;
    pre_un_kps = cur_un_kps;
    cur_img = forw_img;
    cur_kps = forw_kps;
    undistoredPoints();
    prev_img = cur_img;
}

void FeatureTrack::undistoredPoints() {
    cur_un_kps.clear();
    cur_un_kps_map.clear();
    std::cout << "cur_kps size:" << cur_kps.size() << std::endl;
    for (unsigned int i = 0; i < cur_kps.size(); ++i) {
        Eigen::Vector2d cur_pt(cur_kps[i].x, cur_kps[i].y);
        Eigen::Vector3d b;
        camera_ptr->liftProjective(cur_pt, b);
        cur_un_kps.push_back(cv::Point2f(b.x() / b.z(), b.y() / b.z()));
        cur_un_kps_map.insert(std::make_pair(ids[i], cv::Point2f(b.x() / b.z(), b.y() / b.z())));
    }
    // 计算像素点的速度
    if (!pre_un_kps_map.empty()) {
        double dt = cur_time - prev_time;
        pts_velocity.clear();
        for (unsigned int i = 0; i < cur_un_kps.size(); ++i) {
            if (ids[i] != -1) {
                auto it = pre_un_kps_map.find(ids[i]);
                if (it != pre_un_kps_map.end()) {
                    double v_x = (cur_un_kps[i].x - it->second.x) / dt;
                    double v_y = (cur_un_kps[i].y - it->second.y) / dt;
                    pts_velocity.push_back(cv::Point2f(v_x, v_y));
                } else {
                    pts_velocity.push_back(cv::Point2f(0, 0));
                }
            } else {
                pts_velocity.push_back(cv::Point2f(0, 0));
            }
        }
    } else {
        for (unsigned int i = 0; i < cur_kps.size(); i++) {
            pts_velocity.push_back(cv::Point2f(0, 0));
        }
    }
    pre_un_kps_map = cur_un_kps_map;
}

}  // namespace vslam::vins