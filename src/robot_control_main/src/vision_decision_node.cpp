/**
 * @file vision_decision_node.cpp
 * @brief Handles all YOLO/AI vision processing and ROI-based decision making
 * 
 * Responsibilities:
 * - Subscribe to YOLO bounding boxes (cam0, cam1)
 * - Process ROI detection for input tray rows
 * - Process ROI detection for output tray slots
 * - Publish selected row/slot decisions
 * 
 * Topics Published:
 * - /vision/input_tray/selected_row (Int32)
 * - /vision/input_tray/row_status (Int32MultiArray) - 5 values: 0=empty, 1=full
 * - /vision/input_tray/empty (Bool)
 * - /vision/output_tray/selected_slot (Int32)
 * - /vision/output_tray/slot_status (Int32MultiArray) - 9 values: 0=empty, 1=occupied
 * 
 * Topics Subscribed:
 * - cam0HP/yolo/bounding_boxes (Detection2DArray) - Input tray
 * - cam1HP/yolo/bounding_boxes (Detection2DArray) - Output tray
 * - /robot/set_mode (Int32) - To know current mode (1=AUTO, 2=AI, 3=MANUAL)
 */

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/header.hpp"
#include "vision_msgs/msg/detection2_d_array.hpp"


#include <vector>
#include <array>
#include <deque>
#include <algorithm>
#include <cmath>
#include <mutex>
#include <atomic>
#include <chrono>
#include <numeric>

using vision_msgs::msg::Detection2D;
using vision_msgs::msg::Detection2DArray;

// ============================================================================
// UTILITY STRUCTURES
// ============================================================================

struct Point2 {
    float x{}, y{};
};

struct ROIQuad {
    std::array<Point2, 4> pts;
    int min_x{}, max_x{}, min_y{}, max_y{};

    static ROIQuad FromCorners(const std::vector<std::pair<int, int>>& corners) {
        ROIQuad r{};
        for (size_t i = 0; i < 4; ++i) {
            r.pts[i] = Point2{static_cast<float>(corners[i].first),
                              static_cast<float>(corners[i].second)};
        }
        r.min_x = std::min({corners[0].first, corners[1].first,
                            corners[2].first, corners[3].first});
        r.max_x = std::max({corners[0].first, corners[1].first,
                            corners[2].first, corners[3].first});
        r.min_y = std::min({corners[0].second, corners[1].second,
                            corners[2].second, corners[3].second});
        r.max_y = std::max({corners[0].second, corners[1].second,
                            corners[2].second, corners[3].second});
        return r;
    }

    inline bool bbox_contains(float x, float y) const {
        return (x >= min_x && x <= max_x && y >= min_y && y <= max_y);
    }

    inline bool contains(float x, float y) const {
        if (!bbox_contains(x, y)) return false;
        auto cross = [](const Point2& a, const Point2& b, const Point2& c) {
            return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
        };
        const Point2 p{x, y};
        float c0 = cross(pts[0], pts[1], p);
        float c1 = cross(pts[1], pts[2], p);
        float c2 = cross(pts[2], pts[3], p);
        float c3 = cross(pts[3], pts[0], p);
        
        bool all_nonneg = (c0 >= 0 && c1 >= 0 && c2 >= 0 && c3 >= 0);
        bool all_nonpos = (c0 <= 0 && c1 <= 0 && c2 <= 0 && c3 <= 0);
        return all_nonneg || all_nonpos;
    }
};

struct RowFilter {
    size_t window = 3;
    int max_fall = 2;
    int ready_consec = 4;

    std::deque<int> hist;
    int last_filtered = 0;
    int ready_streak = 0;

    int filter_count(int raw_count) {
        hist.push_back(raw_count);
        if (hist.size() > window) hist.pop_front();

        std::vector<int> tmp(hist.begin(), hist.end());
        std::nth_element(tmp.begin(), tmp.begin() + tmp.size() / 2, tmp.end());
        int med = tmp[tmp.size() / 2];

        int y = med;
        if (y < last_filtered - max_fall) {
            y = last_filtered - max_fall;
        }
        last_filtered = y;
        return y;
    }

    bool update_ready(bool raw_ready) {
        if (raw_ready) ready_streak++;
        else ready_streak = 0;
        return ready_streak >= ready_consec;
    }

    void clear() {
        hist.clear();
        last_filtered = 0;
        ready_streak = 0;
    }
};

enum class SlotStableState : int {
    EMPTY = 0,
    OCC_OK = 1,
    MIS = 2
};

// ============================================================================
// DETECTION HELPERS (Nova5-style: NMS + IoU greedy slot assignment)
// ============================================================================

struct Box { double x1, y1, x2, y2; };

static double IoU(const Box& A, const Box& B) {
    double xA = std::max(A.x1, B.x1), yA = std::max(A.y1, B.y1);
    double xB = std::min(A.x2, B.x2), yB = std::min(A.y2, B.y2);
    double inter = std::max(0.0, xB - xA) * std::max(0.0, yB - yA);
    double aA = (A.x2 - A.x1) * (A.y2 - A.y1);
    double aB = (B.x2 - B.x1) * (B.y2 - B.y1);
    return inter / std::max(1e-6, aA + aB - inter);
}

static std::vector<int> nmsGreedy(const std::vector<Box>& boxes,
                                   const std::vector<double>& scores, double iou_thresh) {
    std::vector<int> idx(boxes.size());
    std::iota(idx.begin(), idx.end(), 0);
    std::sort(idx.begin(), idx.end(), [&](int a, int b){ return scores[a] > scores[b]; });
    std::vector<int> keep;
    std::vector<bool> removed(boxes.size(), false);
    for (size_t i = 0; i < idx.size(); ++i) {
        int ia = idx[i];
        if (removed[ia]) continue;
        keep.push_back(ia);
        for (size_t j = i + 1; j < idx.size(); ++j) {
            int ib = idx[j];
            if (!removed[ib] && IoU(boxes[ia], boxes[ib]) >= iou_thresh)
                removed[ib] = true;
        }
    }
    return keep;
}

static int select_contiguous_empty(const std::vector<int>& empty_slots, int total_slots = 9) {
    std::vector<bool> is_empty(total_slots + 1, false);
    for (int id : empty_slots)
        if (id >= 1 && id <= total_slots) is_empty[id] = true;
    for (int i = 1; i <= total_slots; ++i) {
        if (!is_empty[i]) continue;
        bool all_full_before = true;
        for (int j = 1; j < i; ++j)
            if (is_empty[j]) { all_full_before = false; break; }
        return all_full_before ? i : -1;
    }
    return -1;
}

// [HP] System convention: AUTO=1, AI=2, MANUAL=3 (match robot_controller.cpp
// + cartridge_providesystem). Previous enum had MANUAL=0 → mode 3 from GUI
// fell through to default AUTO branch (fill rows true), making AI threshold
// never apply.
enum class ControlMode : uint8_t {
    AUTO   = 1,
    AI     = 2,
    MANUAL = 3
};

// ============================================================================
// VISION DECISION NODE
// ============================================================================

using namespace std::chrono_literals;

class VisionDecisionNode : public rclcpp::Node {
public:
    VisionDecisionNode() : Node("vision_decision_node") {
        RCLCPP_INFO(get_logger(), "[VISION] === Vision Decision Node Starting ===");
        
        initInputTrayROIs();
        initOutputTrayROIs();
        
        // Initialize filters
        row_filters_.assign(5, RowFilter{});
        row_full_.assign(5, false);
        
        // Publishers
        pub_selected_row_ = create_publisher<std_msgs::msg::Int32>(
            "/vision/input_tray/selected_row", 10);
        pub_ai_row_ = create_publisher<std_msgs::msg::Int32>(
            "/camera/ai/selected_row", 10);
        pub_row_status_ = create_publisher<std_msgs::msg::Int32MultiArray>(
            "/vision/input_tray/row_status", 10);
        pub_input_empty_ = create_publisher<std_msgs::msg::Bool>(
            "/vision/input_tray/empty", 10);
        // [HP] Layer-1 tray-presence check: true khi có khay trong khung hình
        pub_input_present_ = create_publisher<std_msgs::msg::Bool>(
            "/vision/input_tray/present", 10);
        pub_selected_slot_ = create_publisher<std_msgs::msg::Int32>(
            "/vision/output_tray/selected_slot", 10);
        pub_ai_slot_ = create_publisher<std_msgs::msg::Int32>(
            "/camera/ai/selected_slot", 10);
        pub_slot_status_ = create_publisher<std_msgs::msg::Int32MultiArray>(
            "/vision/output_tray/slot_status", 10);
        pub_heartbeat_ = create_publisher<std_msgs::msg::Header>(
            "/vision/heartbeat", 10);
        
        // Tray change publishers
        // Camera → Cartridge trực tiếp (AI mode output tray full)
        pub_change_tray_output_ = create_publisher<std_msgs::msg::Bool>(
            "/robot/done_tray_output", 10);
        // Camera → Robot logic (để robot biết tray đang thay)
        pub_output_tray_full_ = create_publisher<std_msgs::msg::Bool>(
            "/vision/output_tray/full", 10);

        
        // Subscriptions with SensorDataQoS for low latency
        sub_cam0_ = create_subscription<Detection2DArray>(
            "cam0HP/yolo/bounding_boxes", rclcpp::SensorDataQoS(),
            std::bind(&VisionDecisionNode::camera1Callback, this, std::placeholders::_1));
            
        sub_cam1_ = create_subscription<Detection2DArray>(
            "cam1HP/yolo/bounding_boxes", rclcpp::SensorDataQoS(),
            std::bind(&VisionDecisionNode::camera2Callback, this, std::placeholders::_1));
        
        // Mode subscription
        sub_mode_ = create_subscription<std_msgs::msg::Int32>(
            "/robot/set_mode", 10,
            [this](const std_msgs::msg::Int32::SharedPtr msg) {
                current_mode_ = static_cast<ControlMode>(msg->data);
                RCLCPP_INFO(get_logger(), "[VISION] Mode changed to: %d", msg->data);
            });

        // Heartbeat Timer
        heartbeat_timer_ = create_wall_timer(500ms, [this]() {
            std_msgs::msg::Header h;
            h.stamp = this->now();
            h.frame_id = "vision_decision";
            pub_heartbeat_->publish(h);
        });



        RCLCPP_INFO(get_logger(), "[VISION] === Vision Decision Node Ready ===");
    }

private:
    // ========================================================================
    // CONSTANTS
    // ========================================================================
    static constexpr int    INPUT_ROW_THRESHOLD    = 8;
    static constexpr float  DETECTION_SCORE_THRESH = 0.60f;
    static constexpr int    SLOT_CONFIRM_FRAMES    = 2;
    static constexpr size_t N_OUTPUT_SLOTS         = 9;  // must match output_tray_rois_ size

    // ========================================================================
    // MODE STATE
    // ========================================================================
    // [HP] Default AI để khi vision khởi động trước GUI mode publish, vẫn dùng
    // YOLO threshold thay vì AUTO branch (fill rows true bỏ qua YOLO).
    std::atomic<ControlMode> current_mode_{ControlMode::AI};

    // ========================================================================
    // INPUT TRAY STATE
    // ========================================================================
    std::vector<ROIQuad> input_tray_rois_;
    std::vector<RowFilter> row_filters_;
    std::vector<bool> row_full_;
    std::atomic<bool> input_tray_empty_{false};
    int selected_input_row_{-1};
    // [HP] Layer-1: tray-present ROI (bao toàn bộ khay) + state
    ROIQuad input_tray_outer_roi_;
    std::atomic<bool> input_tray_present_{false};

    // ========================================================================
    // OUTPUT TRAY STATE
    // ========================================================================
    std::array<ROIQuad, 9> output_tray_rois_;
    std::array<SlotStableState, 9> slot_stable_state_;
    std::array<int, 9> slot_empty_streak_;
    std::array<int, 9> slot_occ_streak_;
    std::array<int, 9> slot_mis_streak_;
    int selected_output_slot_{-1};
    std::mutex slot_detection_mutex_;

    // ========================================================================
    // PERFORMANCE TRACKING
    // ========================================================================
    std::atomic<uint64_t> callback_count_{0};
    std::atomic<uint64_t> total_callback_time_us_{0};

    // ========================================================================
    // ROS INTERFACES
    // ========================================================================
    rclcpp::Subscription<Detection2DArray>::SharedPtr sub_cam0_;
    rclcpp::Subscription<Detection2DArray>::SharedPtr sub_cam1_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub_mode_;
    
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub_selected_row_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub_ai_row_;
    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr pub_row_status_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_input_empty_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_input_present_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub_selected_slot_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub_ai_slot_;
    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr pub_slot_status_;
    rclcpp::Publisher<std_msgs::msg::Header>::SharedPtr pub_heartbeat_;
    
    // Tray change publishers
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_change_tray_output_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_output_tray_full_;
    
    // Debounce counters for tray change
    std::atomic<int> output_full_streak_{0};
    std::atomic<bool> tray_output_change_sent_{false};
    static constexpr int TRAY_CHANGE_CONFIRM_FRAMES = 10;

    rclcpp::TimerBase::SharedPtr heartbeat_timer_;

    // ========================================================================
    // INITIALIZATION
    // ========================================================================
    void initInputTrayROIs() {
        // [HP] Layer-1 outer TRAY ROI — bao toàn bộ khay (12 cartridge × 5 row).
        // Toạ độ pick từ roi_picker_web.py @ production 1280×720, 6/2026.
        std::vector<std::pair<int, int>> tray_corners = {
            {309, 621}, {342, 66}, {1112, 62}, {1152, 632}
        };
        input_tray_outer_roi_ = ROIQuad::FromCorners(tray_corners);

        // [HP] Layer-2 row ROIs — 5 rows trong khay.
        // Toạ độ pick từ roi_picker_web.py @ production 1280×720, 6/2026.
        std::vector<std::vector<std::pair<int, int>>> row_corners = {
            {{357, 585}, {380, 107}, {508, 106}, {496, 584}},   // ROW1
            {{508, 585}, {519, 105}, {639, 106}, {637, 586}},   // ROW2
            {{653, 590}, {655, 105}, {772, 105}, {782, 590}},   // ROW3
            {{801, 590}, {793, 103}, {914, 105}, {933, 594}},   // ROW4
            {{946, 594}, {930, 103}, {1054, 102}, {1082, 592}}  // ROW5
        };

        input_tray_rois_.clear();
        for (const auto& corners : row_corners) {
            input_tray_rois_.push_back(ROIQuad::FromCorners(corners));
        }
        RCLCPP_INFO(get_logger(), "[VISION] Initialized TRAY ROI + %zu row ROIs",
                    input_tray_rois_.size());
    }

    void initOutputTrayROIs() {
        std::vector<std::vector<std::pair<int, int>>> slot_corners = {
            {{280, 505}, {280, 415}, {580, 430}, {584, 519}},
            {{275, 415}, {295, 319}, {579, 327}, {576, 424}},
            {{297, 315}, {317, 228}, {582, 229}, {580, 320}},
            {{349, 249}, {362, 172}, {619, 168}, {625, 249}},
            {{585, 527}, {590, 242}, {690, 240}, {690, 525}},
            {{699, 517}, {696, 423}, {1010, 415}, {1026, 502}},
            {{696, 420}, {700, 328}, {989, 328}, {1011, 405}},
            {{694, 323}, {696, 230}, {966, 230}, {985, 325}},
        };
        
        for (size_t i = 0; i < slot_corners.size() && i < output_tray_rois_.size(); ++i) {
            output_tray_rois_[i] = ROIQuad::FromCorners(slot_corners[i]);
        }

        slot_stable_state_.fill(SlotStableState::EMPTY);
        slot_empty_streak_.fill(0);
        slot_occ_streak_.fill(0);
        slot_mis_streak_.fill(0);

        // Slots without a defined ROI are treated as occupied so they are
        // never selected by select_contiguous_empty (avoids ghost-slot bug).
        for (size_t i = slot_corners.size(); i < N_OUTPUT_SLOTS; ++i)
            slot_stable_state_[i] = SlotStableState::OCC_OK;

        RCLCPP_INFO(get_logger(), "[VISION] Initialized %zu/%zu output tray slots",
                    slot_corners.size(), N_OUTPUT_SLOTS);
    }

    // ========================================================================
    // INPUT TRAY CALLBACK (from robot_logic_node.cpp)
    // ========================================================================
    void camera1Callback(const Detection2DArray::SharedPtr msg) {
        auto start = std::chrono::high_resolution_clock::now();
        
        if (current_mode_ == ControlMode::MANUAL) return;

        bool use_ai = (current_mode_ == ControlMode::AI);

        if (!use_ai) {
            // AUTO: skip YOLO entirely, assume tray present + all rows full
            input_tray_present_.store(true);
            std::fill(row_full_.begin(), row_full_.end(), true);
        } else {
            // [HP Layer-1] TRAY ROI = spatial filter — chỉ giữ detection BÊN TRONG
            // khay vận hành. Loại nhiễu/khay khác cạnh ra khỏi row counting.
            // [HP Layer-2] Row counting áp dụng RowFilter cho stability.
            std::vector<int> row_counts(5, 0);
            int total_in_tray = 0;

            for (const auto& det : msg->detections) {
                if (det.results.empty()) continue;
                const std::string& class_id = det.results[0].hypothesis.class_id;
                float score = det.results[0].hypothesis.score;
                // [HP HEF6] class "1" = CARTRIDGE (class "0" = whole TRAY bbox, ignore).
                if (class_id != "1" || score < DETECTION_SCORE_THRESH) continue;
                float cx = det.bbox.center.position.x;
                float cy = det.bbox.center.position.y;

                // L1: spatial filter — bỏ nếu nằm ngoài TRAY ROI
                if (!input_tray_outer_roi_.bbox_contains(cx, cy)) continue;
                if (!input_tray_outer_roi_.contains(cx, cy)) continue;
                total_in_tray++;

                // L2: tìm row ROI khớp
                for (size_t i = 0; i < input_tray_rois_.size(); ++i) {
                    if (!input_tray_rois_[i].bbox_contains(cx, cy)) continue;
                    if (input_tray_rois_[i].contains(cx, cy)) { row_counts[i]++; break; }
                }
            }

            // Tray present nếu có bất kỳ detection nào hợp lệ trong TRAY ROI.
            input_tray_present_.store(total_in_tray > 0);

            for (size_t i = 0; i < 5; ++i) {
                int raw_count  = row_counts[i];
                int filtered   = row_filters_[i].filter_count(raw_count);
                // [HP] Row READY chỉ khi ĐÚNG bằng sức chứa (8). Một hàng vật lý
                // chứa tối đa 8 cartridge → count > 8 là nhân đôi/nhiễu detection,
                // count < 8 là hàng chưa đầy. Cả hai trường hợp → NOT ready (bỏ qua).
                bool raw_ready = (raw_count == INPUT_ROW_THRESHOLD);
                bool stable    = row_filters_[i].update_ready(raw_ready);
                row_full_[i]   = stable;
                RCLCPP_INFO(get_logger(),
                    "[ROW %zu] raw=%d filtered=%d thr=%d READY(raw)=%s READY(stable)=%s (streak=%d/%d) [tray_total=%d]",
                    i + 1, raw_count, filtered, INPUT_ROW_THRESHOLD,
                    raw_ready ? "YES" : "NO", stable ? "YES" : "NO",
                    row_filters_[i].ready_streak, row_filters_[i].ready_consec,
                    total_in_tray);
            }
        }
        
        input_tray_empty_ = std::none_of(row_full_.begin(), row_full_.end(),
                                          [](bool full) { return full; });
        
        // Find first available (full) row for picking
        selected_input_row_ = -1;
        for (size_t i = 0; i < row_full_.size(); ++i) {
            if (row_full_[i]) {
                selected_input_row_ = static_cast<int>(i + 1);  // 1-indexed
                break;
            }
        }
        
        // Publish results
        auto row_msg = std_msgs::msg::Int32();
        row_msg.data = selected_input_row_;
        pub_selected_row_->publish(row_msg);
        pub_ai_row_->publish(row_msg);
        
        auto empty_msg = std_msgs::msg::Bool();
        empty_msg.data = input_tray_empty_;
        pub_input_empty_->publish(empty_msg);

        auto present_msg = std_msgs::msg::Bool();
        present_msg.data = input_tray_present_.load();
        pub_input_present_->publish(present_msg);
        
        auto status_msg = std_msgs::msg::Int32MultiArray();
        status_msg.data.resize(5);
        for (size_t i = 0; i < 5; ++i) {
            status_msg.data[i] = row_full_[i] ? 1 : 0;
        }
        pub_row_status_->publish(status_msg);

        
        // Performance tracking
        auto end = std::chrono::high_resolution_clock::now();
        auto duration_us = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
        callback_count_.fetch_add(1);
        total_callback_time_us_.fetch_add(duration_us);
        
        if (callback_count_ % 100 == 0) {
            uint64_t avg_us = total_callback_time_us_ / callback_count_;
            RCLCPP_INFO(get_logger(), "[PERF] Cam0 avg: %lu µs", avg_us);
        }
    }

    // ========================================================================
    // OUTPUT TRAY CALLBACK (from robot_logic_node.cpp)
    // ========================================================================
    void camera2Callback(const Detection2DArray::SharedPtr msg) {
        if (current_mode_ == ControlMode::MANUAL) return;
        
        if (current_mode_ == ControlMode::AUTO) {
            // AUTO: Robot logic tự quản lý slot, camera không can thiệp
            return;
        }

        // Step 1: NMS + IoU greedy slot assignment (Nova5-style)
        struct DetRec { Box box; double score; };
        std::vector<DetRec> dets_raw;
        for (const auto& det : msg->detections) {
            if (det.results.empty()) continue;
            int cid = -1;
            try { cid = std::stoi(det.results[0].hypothesis.class_id); } catch (...) { continue; }
            if (cid != 1 && cid != 2 && cid != 3) continue;
            double score = det.results[0].hypothesis.score;
            if (score < DETECTION_SCORE_THRESH) continue;
            double cx = det.bbox.center.position.x;
            double cy = det.bbox.center.position.y;
            double hw = det.bbox.size_x / 2.0, hh = det.bbox.size_y / 2.0;
            dets_raw.push_back({{cx - hw, cy - hh, cx + hw, cy + hh}, score});
        }
        std::vector<Box> nms_boxes;
        std::vector<double> nms_scores;
        for (auto& d : dets_raw) { nms_boxes.push_back(d.box); nms_scores.push_back(d.score); }
        auto kept = nmsGreedy(nms_boxes, nms_scores, 0.45);

        // Build candidates sorted by match score
        struct Cand { int slot; int ki; double sc; };
        std::vector<Cand> cands;
        const size_t n_slots = N_OUTPUT_SLOTS;
        for (size_t s = 0; s < n_slots; ++s) {
            const auto& roi = output_tray_rois_[s];
            Box slot_box{(double)roi.min_x, (double)roi.min_y,
                         (double)roi.max_x, (double)roi.max_y};
            for (size_t ki = 0; ki < kept.size(); ++ki) {
                const Box& db = nms_boxes[kept[ki]];
                double iou = IoU(slot_box, db);
                float dcx = (float)((db.x1 + db.x2) / 2.0);
                float dcy = (float)((db.y1 + db.y2) / 2.0);
                bool inside = roi.contains(dcx, dcy);
                if (iou < 0.1 && !inside) continue;
                cands.push_back({(int)s, (int)ki, iou + (inside ? 1.0 : 0.0)});
            }
        }
        std::sort(cands.begin(), cands.end(),
                  [](const Cand& a, const Cand& b){ return a.sc > b.sc; });

        std::array<SlotStableState, 9> instant_state;
        instant_state.fill(SlotStableState::EMPTY);
        std::vector<bool> slot_used(n_slots, false);
        std::vector<bool> det_used(kept.size(), false);
        for (auto& c : cands) {
            if (slot_used[c.slot] || det_used[c.ki]) continue;
            instant_state[c.slot] = SlotStableState::OCC_OK;
            slot_used[c.slot] = true;
            det_used[c.ki] = true;
        }
        
        // Step 2: Update debouncing with mutex
        int local_selected_slot = -1;
        
        {
            std::lock_guard<std::mutex> lock(slot_detection_mutex_);
            
            for (size_t s = 0; s < N_OUTPUT_SLOTS; ++s) {
                if (instant_state[s] == SlotStableState::EMPTY) {
                    slot_empty_streak_[s]++;
                    slot_occ_streak_[s] = 0;
                    slot_mis_streak_[s] = 0;
                    if (slot_empty_streak_[s] >= SLOT_CONFIRM_FRAMES)
                        slot_stable_state_[s] = SlotStableState::EMPTY;
                } else if (instant_state[s] == SlotStableState::OCC_OK) {
                    slot_occ_streak_[s]++;
                    slot_empty_streak_[s] = 0;
                    slot_mis_streak_[s] = 0;
                    if (slot_occ_streak_[s] >= SLOT_CONFIRM_FRAMES)
                        slot_stable_state_[s] = SlotStableState::OCC_OK;
                } else {
                    slot_mis_streak_[s]++;
                    slot_empty_streak_[s] = 0;
                    slot_occ_streak_[s] = 0;
                    if (slot_mis_streak_[s] >= SLOT_CONFIRM_FRAMES)
                        slot_stable_state_[s] = SlotStableState::MIS;
                }
            }

            // Select first EMPTY slot (contiguous fill) — Nova5-style
            std::vector<int> empty_slots;
            for (size_t i = 0; i < N_OUTPUT_SLOTS; ++i)
                if (slot_stable_state_[i] == SlotStableState::EMPTY)
                    empty_slots.push_back(static_cast<int>(i + 1));
            local_selected_slot = select_contiguous_empty(
                empty_slots, static_cast<int>(N_OUTPUT_SLOTS));
        }
        
        selected_output_slot_ = local_selected_slot;
        
        // Publish results
        auto slot_msg = std_msgs::msg::Int32();
        slot_msg.data = local_selected_slot;
        pub_selected_slot_->publish(slot_msg);
        pub_ai_slot_->publish(slot_msg);
        
        auto status_msg = std_msgs::msg::Int32MultiArray();
        status_msg.data.resize(N_OUTPUT_SLOTS);
        for (size_t i = 0; i < N_OUTPUT_SLOTS; ++i) {
            status_msg.data[i] = (slot_stable_state_[i] == SlotStableState::EMPTY) ? 0 : 1;
        }
        pub_slot_status_->publish(status_msg);
        
        // Output tray full detection with debounce
        bool output_full = (local_selected_slot == -1);  // No empty slot found
        if (output_full) {
            output_full_streak_++;
            if (output_full_streak_ >= TRAY_CHANGE_CONFIRM_FRAMES && !tray_output_change_sent_) {
                auto change_msg = std_msgs::msg::Bool();
                change_msg.data = true;
                // Gửi trực tiếp tới cartridge system
                pub_change_tray_output_->publish(change_msg);
                // Thông báo robot logic để set waiting_for_new_output_
                pub_output_tray_full_->publish(change_msg);
                tray_output_change_sent_ = true;
                RCLCPP_WARN(get_logger(), "[VISION] 📦 OUTPUT TRAY FULL - Change tray signal sent to cartridge + robot");
            }
        } else {
            output_full_streak_ = 0;
            tray_output_change_sent_ = false;  // Reset when new tray placed
        }
    }
};

// ============================================================================
// MAIN
// ============================================================================

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<VisionDecisionNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
