// DETECTION
    // ================================================================

    void detectObstacle(
        const sensor_msgs::msg::LaserScan &msg)
    {
        std::vector<Point> candidates;


        // ============================================================
        // CHECK EVERY LASER POINT
        // ============================================================

        for (size_t i = 0;
             i < msg.ranges.size();
             ++i)
        {
            double range =
                static_cast<double>(
                    msg.ranges[i]);


            // --------------------------------------------------------
            // INVALID RANGE
            // --------------------------------------------------------

            if (!std::isfinite(range))
            {
                continue;
            }


            if (range < min_range_ ||
                range > max_range_)
            {
                continue;
            }


            // --------------------------------------------------------
            // ANGLE
            // --------------------------------------------------------

            double angle =
                msg.angle_min +
                static_cast<double>(i) *
                msg.angle_increment;


            // --------------------------------------------------------
            // CALCULATE IDEAL BASELINE
            // --------------------------------------------------------

            double normal_range;


            if (!calculateBaselineRange(
                    angle,
                    normal_range))
            {
                continue;
            }


            // --------------------------------------------------------
            // CURRENT XY
            // --------------------------------------------------------

            double x =
                range *
                std::cos(angle);

            double y =
                range *
                std::sin(angle);


            // --------------------------------------------------------
            // DETECTION BOX
            // --------------------------------------------------------

            if (x < min_x_ ||
                x > max_x_)
            {
                continue;
            }


            if (y < min_y_ ||
                y > max_y_)
            {
                continue;
            }


            // --------------------------------------------------------
            // BASELINE XY
            // --------------------------------------------------------

            double baseline_x =
                normal_range *
                std::cos(angle);

            double baseline_y =
                normal_range *
                std::sin(angle);


            // --------------------------------------------------------
            // COMPARE RANGE
            // --------------------------------------------------------

            double difference =
                normal_range -
                range;


            // --------------------------------------------------------
            // DEBUG
            // --------------------------------------------------------

            if (difference >
                obstacle_threshold_)
            {
                RCLCPP_INFO_THROTTLE(
                    this->get_logger(),
                    *this->get_clock(),
                    500,

                    "CANDIDATE index=%zu "
                    "angle=%.2f deg "
                    "current=%.3f "
                    "baseline=%.3f "
                    "diff=%.3f "
                    "x=%.3f "
                    "y=%.3f",

                    i,

                    angle *
                    180.0 /
                    M_PI,

                    range,

                    normal_range,

                    difference,

                    x,

                    y);
            }


            // --------------------------------------------------------
            // OBSTACLE CANDIDATE
            // --------------------------------------------------------

            if (difference <
                obstacle_threshold_)
            {
                continue;
            }


            candidates.push_back(
                {
                    i,

                    angle,

                    range,

                    x,

                    y,

                    normal_range,

                    baseline_y,

                    difference
                });
        }


        // ============================================================
        // NO CANDIDATES
        // ============================================================

        if (candidates.empty())
        {
            publishEmptyObstacleScan(msg);

            publishBox(
                msg.header.frame_id);

            return;
        }


        // ============================================================
        // GROUP POINTS
        // ============================================================

        std::vector<
            std::vector<Point>
        > groups;


        std::vector<Point>
            current_group;


        current_group.push_back(
            candidates[0]);


        for (size_t i = 1;
             i < candidates.size();
             ++i)
        {
            size_t gap =
                candidates[i].index -
                candidates[i - 1].index;


            if (gap <=
                static_cast<size_t>(
                    max_index_gap_))
            {
                current_group.push_back(
                    candidates[i]);
            }
            else
            {
                groups.push_back(
                    current_group);

                current_group.clear();

                current_group.push_back(
                    candidates[i]);
            }
        }


        if (!current_group.empty())
        {
            groups.push_back(
                current_group);
        }


        // ============================================================
        // FIND BEST GROUP
        // ============================================================

        const std::vector<Point>*
            best_group = nullptr;


        double best_score = 0.0;


        for (const auto &group :
             groups)
        {
            if (group.size() <
                static_cast<size_t>(
                    min_points_))
            {
                continue;
            }


            double max_diff = 0.0;


            for (const auto &p :
                 group)
            {
                max_diff =
                    std::max(
                        max_diff,
                        p.difference);
            }


            // Score:
            //
            // number of points
            // × maximum deviation

            double score =
                static_cast<double>(
                    group.size()) *
                max_diff;


            if (best_group == nullptr ||
                score > best_score)
            {
                best_group =
                    &group;

                best_score =
                    score;
            }
        }


        // ============================================================
        // NO VALID GROUP
        // ============================================================

        if (best_group == nullptr)
        {
            publishEmptyObstacleScan(msg);

            publishBox(
                msg.header.frame_id);

            return;
        }


        // ============================================================
        // OUTPUT SCAN
        // ============================================================

        auto obstacle_scan =
            msg;


        for (auto &r :
             obstacle_scan.ranges)
        {
            r =
                std::numeric_limits<float>
                ::infinity();
        }


        // ------------------------------------------------------------
        // PUT DETECTED POINTS
        // ------------------------------------------------------------

        for (const auto &point :
             *best_group)
        {
            obstacle_scan
                .ranges[point.index] =
                static_cast<float>(
                    point.range);
        }


        obstacle_scan_pub_->publish(
            obstacle_scan);


        // ============================================================
        // BOUNDARY
        // ============================================================

        const Point &first =
            best_group->front();

        const Point &last =
            best_group->back();


        double min_x =
            first.x;

        double max_x =
            first.x;

        double min_y =
            first.y;

        double max_y =
            first.y;


        double maximum_difference =
            0.0;


        size_t maximum_difference_index =
            first.index;


        // ------------------------------------------------------------
        // CALCULATE BOUNDARY
        // ------------------------------------------------------------

        for (const auto &point :
             *best_group)
        {
            min_x =
                std::min(
                    min_x,
                    point.x);

            max_x =
                std::max(
                    max_x,
                    point.x);

            min_y =
                std::min(
                    min_y,
                    point.y);

            max_y =
                std::max(
                    max_y,
                    point.y);


            if (point.difference >
                maximum_difference)
            {
                maximum_difference =
                    point.difference;

                maximum_difference_index =
                    point.index;
            }
        }


        // ============================================================
        // RESULT
        // ============================================================

        double max_angle =
            msg.angle_min +
            maximum_difference_index *
            msg.angle_increment;


        RCLCPP_INFO(
            this->get_logger(),
            "==========================================");

        RCLCPP_INFO(
            this->get_logger(),
            "       OBSTACLE DETECTED");

        RCLCPP_INFO(
            this->get_logger(),
            "==========================================");


        RCLCPP_INFO(
            this->get_logger(),
            "Number of points : %zu",
            best_group->size());


        RCLCPP_INFO(
            this->get_logger(),
            "Start index      : %zu",
            first.index);


        RCLCPP_INFO(
            this->get_logger(),
            "End index        : %zu",
            last.index);


        RCLCPP_INFO(
            this->get_logger(),
            "Start angle      : %.2f deg",
            first.angle *
            180.0 /
            M_PI);


        RCLCPP_INFO(
            this->get_logger(),
            "End angle        : %.2f deg",
            last.angle *
            180.0 /
            M_PI);


        RCLCPP_INFO(
            this->get_logger(),
            "Max deviation index : %zu",
            maximum_difference_index);


        RCLCPP_INFO(
            this->get_logger(),
            "Max deviation angle : %.2f deg",
            max_angle *
            180.0 /
            M_PI);


        RCLCPP_INFO(
            this->get_logger(),
            "X range          : %.3f -> %.3f m",
            min_x,
            max_x);


        RCLCPP_INFO(
            this->get_logger(),
            "Y range          : %.3f -> %.3f m",
            min_y,
            max_y);


        RCLCPP_INFO(
            this->get_logger(),
            "Maximum deviation: %.3f m",
            maximum_difference);


        RCLCPP_INFO(
            this->get_logger(),
            "Baseline Y       : %.3f m",
            baseline_y_ +
            baseline_offset_);


        RCLCPP_INFO(
            this->get_logger(),
            "==========================================");


        publishBox(
            msg.header.frame_id);
    }