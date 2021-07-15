#include "objclassifier.h"

ObjClassiferLidar::ObjClassiferLidar(){
    setParams();

    static ros::Subscriber track_bbox_sub = nh.subscribe ("tracker/tracked_bboxes", 1, &ObjClassiferLidar::trackboxCallback, this);

    obj_info_pub      = nh.advertise<obj_msgs::ObjList>("obj_info", 1);
    local_link_pc_pub = nh.advertise<sensor_msgs::PointCloud2>("local_link_points",1);

    ego_markers_pub   = nh.advertise<visualization_msgs::MarkerArray>("classifier/ego_state", 1);
    obj_id_marker_pub = nh.advertise<visualization_msgs::MarkerArray>("classifier/Id_markers", 1);
    obj_velocity_marker_pub  = nh.advertise<visualization_msgs::MarkerArray>("classifier/velocity_markers",1);
    obj_velocity_compare_pub = nh.advertise<visualization_msgs::MarkerArray>("classifier/velocity_compare",1);
    obj_boundary_marker_pub  = nh.advertise<visualization_msgs::MarkerArray>("classifier/tiltied_boundary_markers",1);
    obj_rawdetect_pub = nh.advertise<visualization_msgs::MarkerArray>("classifier/raw_boundary_markers",1);
}

void ObjClassiferLidar::setParams(){

    nh.param<double>("/Ego/distance_to_egotail", dist2egotail, 2.0);
    ROS_INFO_STREAM("distance_to_egotail: " << dist2egotail);

    nh.param<double>("/Ego/static_speed_threshold", staticThd, 3.0);
    ROS_INFO_STREAM("static_speed_threshold: "<< staticThd);

    nh.param<bool>("/Ego/use_velocity_fitting", use_velocity_fitting, true);
    ROS_INFO_STREAM("use_velocity_fitting: " << use_velocity_fitting);

    nh.param<double>("/Ego/velocity_fitting_high_thd",velocity_fitting_high_thd, 45);
    ROS_INFO_STREAM("velocity_fitting_high_thd: "<< velocity_fitting_high_thd);

    nh.param<double>("/Ego/velocity_fitting_low_thd",velocity_fitting_low_thd, 10);
    ROS_INFO_STREAM("velocity_fitting_low_thd: "<< velocity_fitting_low_thd);

    nh.param<double>("/Ego/width", egoWidth, 1.0);
    nh.param<double>("/Ego/Length", egoLength, 1.0);
}

void ObjClassiferLidar::trackboxCallback(obj_msgs::ObjTrackBoxes tracked_boxes){
    std::clock_t start_time;
    std::clock_t end_time;
    start_time = std::clock();
    pcl::PointCloud<pcl::PointXYZI>::Ptr temp_pc(new pcl::PointCloud<pcl::PointXYZI>);

    /////////////////////////////
    // Set Parameters from Msg //
    /////////////////////////////
    pcl::fromROSMsg(tracked_boxes.local_link_points, *temp_pc);
    link_pnts = *temp_pc;
    if (link_pnts.size() == 0){
        ROS_WARN_STREAM("NO LINK POINTS");
        return;
    }
    ego_link_point = tracked_boxes.ego_link_point;
    ego_link_type = tracked_boxes.ego_link_type;
    ego_link_type_temp = tracked_boxes.ego_link_type;
    global_yaw_ref_map = tracked_boxes.global_yaw_ref_map;
    ego_link_diff_theta = getNeighborPntsTheta(link_pnts, ego_link_point.x, ego_link_point.y);
    ego_current_speed = tracked_boxes.vehicle_speed;
    ego_yaw_rate = tracked_boxes.vehicle_yaw_rate;
    std::cout<<"EGO LINKT  : "<< ego_link_type<<std::endl;
    std::cout<<"EGO THETA  : "<< -ego_link_diff_theta*180.0/M_PI<<std::endl;
    std::cout<<"EGO SPEED  : "<< ego_current_speed<<std::endl;
    std::cout<<"EGO YAWRATE: "<< ego_yaw_rate <<std::endl;
    local_link_pc_pub.publish(tracked_boxes.local_link_points);

    static float prev_yaw = global_yaw_ref_map;
    diff_yaw = global_yaw_ref_map - prev_yaw;
    // std::cout<<"current_yaw  : "<<global_yaw_ref_map<< std::endl;
    // std::cout<<"previous_yaw : "<<prev_yaw<< std::endl;
    // std::cout<<"yaw rate     : "<<diff_yaw <<std::endl;


    ///////////////////
    // Main Function //
    ///////////////////

    objsPublisher(tracked_boxes);
    prev_yaw = global_yaw_ref_map;

    end_time = std::clock();
    // std::cout << "classifier Running Time  : " << static_cast<double>((end_time-start_time))/CLOCKS_PER_SEC << std::endl;
    return;
}


void ObjClassiferLidar::objsPublisher(obj_msgs::ObjTrackBoxes &tracked_boxes){
    obj_msgs::ObjList classified_objs;
    classified_objs.header.frame_id = tracked_boxes.header.frame_id;
    classified_objs.header.stamp = ros::Time::now();

    visualization_msgs::MarkerArray tilted_bboxes;
    visualization_msgs::MarkerArray velocity_markers;
    visualization_msgs::MarkerArray velocity_compare_markers;
    visualization_msgs::MarkerArray text_markers;
    visualization_msgs::MarkerArray raw_bboxes;
    if (tracked_boxes.bounding_boxes.size() == 0){
        // ROS_WARN_STREAM("There is no Tracked Object!");
        classified_objs.objlist.clear();
    }
    else{
        for (int objNum = 0; objNum < tracked_boxes.bounding_boxes.size(); objNum++){
            double origin_velox= tracked_boxes.bounding_boxes[objNum].xdelta;
            double origin_veloy = tracked_boxes.bounding_boxes[objNum].ydelta;

            obj_msgs::Obj classified_obj = ConvertBoxtoObj(tracked_boxes.bounding_boxes[objNum]);
            classified_obj.type = DefineObjType(classified_obj);

            double fitted_velox = classified_obj.velocity.x;
            double fitted_veloy = classified_obj.velocity.y;

            ///////////////////////////
            ////TEXT MARKER: OBJ ID////
            ///////////////////////////
            visualization_msgs::Marker ul_point;
            ul_point.header.frame_id = tracked_boxes.header.frame_id;
            ul_point.header.stamp = tracked_boxes.header.stamp;
            ul_point.id = 4000+objNum;
            ul_point.type = visualization_msgs::Marker::SPHERE;
            ul_point.action = visualization_msgs::Marker::ADD;
            ul_point.lifetime = ros::Duration(0.1);
            ul_point.pose.orientation.w = 1.0;
            ul_point.pose.position.x = classified_obj.ul.x;
            ul_point.pose.position.y = classified_obj.ul.y;
            ul_point.pose.position.z = 0;
            ul_point.scale.x = 1.0;
            ul_point.scale.y = 1.0;
            ul_point.scale.z = 1.0;
            ul_point.color.r = 1.0;
            ul_point.color.g = 1.0;
            ul_point.color.b = 1.0;
            ul_point.color.a = 1.0;

            visualization_msgs::Marker text_marker;
            text_marker.header.frame_id = tracked_boxes.header.frame_id;
            text_marker.header.stamp = tracked_boxes.header.stamp;
            text_marker.id = 1000+objNum;
            text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
            text_marker.action = visualization_msgs::Marker::ADD;
            text_marker.lifetime = ros::Duration(0.1);
            double orgin_velo = sqrt(origin_velox*origin_velox + origin_veloy*origin_veloy);
            double fitted_velo = sqrt(fitted_velox * fitted_velox + fitted_veloy*fitted_veloy);
            text_marker.text = "ID: " + std::to_string(classified_obj.id) + "\n\n\n" +"origin-> " +std::to_string(orgin_velo*3.6) + "\n" + "fitted-> " + std::to_string(fitted_velo*3.6);

            // text_marker.text = "ID: " + std::to_string(classified_obj.id) + "\n" + tracked_boxes.bounding_boxes[objNum].Class;
            text_marker.pose.orientation.w = 1.0;
            text_marker.pose.position = classified_obj.pose.position;
            text_marker.scale.z = 1.0;
            text_marker.color.r = 1.0;
            text_marker.color.g = 1.0;
            text_marker.color.b = 1.0;
            text_marker.color.a = 1.0;

            ////////////////
            ////VELOCITY////
            ////////////////
            visualization_msgs::Marker velocity_marker;
            velocity_marker.header.frame_id = tracked_boxes.header.frame_id;
            velocity_marker.header.stamp = ros::Time::now();
            velocity_marker.id =2000+objNum;
            velocity_marker.type = visualization_msgs::Marker::ARROW;
            velocity_marker.action = visualization_msgs::Marker::ADD;
            velocity_marker.lifetime = ros::Duration(0.1);
            velocity_marker.pose.orientation.w = 1.0;
            velocity_marker.points.resize(2);
            velocity_marker.points[0].x = classified_obj.pose.position.x;
            velocity_marker.points[0].y = classified_obj.pose.position.y;
            velocity_marker.points[0].z = classified_obj.pose.position.z;
            // ROS_WARN_STREAM(classified_obj.velocity.x << " " <<classified_obj.velocity.y);
            velocity_marker.points[1].x = classified_obj.pose.position.x + classified_obj.velocity.x;// * 10.0;///delta_time;
            velocity_marker.points[1].y = classified_obj.pose.position.y + classified_obj.velocity.y;// * 10.0;///delta_time;
            velocity_marker.points[1].z = classified_obj.pose.position.z + classified_obj.velocity.z;
            velocity_marker.scale.x = 2.0;
            velocity_marker.scale.y = 2.0;
            velocity_marker.scale.z = 0;
            velocity_marker.color.a = 1.0;

            ////////////////
            ////VELOCITY////
            ////////////////
            visualization_msgs::Marker velocity_marker2;
            velocity_marker2.header.frame_id = tracked_boxes.header.frame_id;
            velocity_marker2.header.stamp = ros::Time::now();
            velocity_marker2.id =2500+objNum;
            velocity_marker2.type = visualization_msgs::Marker::ARROW;
            velocity_marker2.action = visualization_msgs::Marker::ADD;
            velocity_marker2.lifetime = ros::Duration(0.1);
            velocity_marker2.pose.orientation.w = 1.0;
            velocity_marker2.points.resize(2);
            velocity_marker2.points[0].x = classified_obj.pose.position.x;
            velocity_marker2.points[0].y = classified_obj.pose.position.y;
            velocity_marker2.points[0].z = classified_obj.pose.position.z;
            // ROS_WARN_STREAM(classified_obj.velocity.x << " " <<classified_obj.velocity.y);
            velocity_marker2.points[1].x = classified_obj.pose.position.x + classified_obj.velocity_compare.x;// * 10.0;///delta_time;
            velocity_marker2.points[1].y = classified_obj.pose.position.y + classified_obj.velocity_compare.y;// * 10.0;///delta_time;
            velocity_marker2.points[1].z = classified_obj.pose.position.z + classified_obj.velocity_compare.z;
            velocity_marker2.scale.x = 1.0;
            velocity_marker2.scale.y = 3.0;
            velocity_marker2.scale.z = 0;
            velocity_marker2.color.r = 0.3;
            velocity_marker2.color.g = 0.3;
            velocity_marker2.color.b = 0.3;
            velocity_marker2.color.a = 1.0;
            ////////////////
            ////BOUDNARY////
            ////////////////
            visualization_msgs::Marker tilted_bbox;
            tilted_bbox.header.frame_id = tracked_boxes.header.frame_id;
            tilted_bbox.header.stamp = ros::Time::now();
            tilted_bbox.id = 3000+objNum;
            tilted_bbox.type = visualization_msgs::Marker::LINE_STRIP;
            tilted_bbox.action = visualization_msgs::Marker::ADD;
            tilted_bbox.lifetime = ros::Duration(0.1);
            std::vector<geometry_msgs::Point> box_points;
            box_points.push_back(classified_obj.ul);
            box_points.push_back(classified_obj.ur);
            box_points.push_back(classified_obj.br);
            box_points.push_back(classified_obj.bl);
            box_points.push_back(classified_obj.ul);
            tilted_bbox.points = box_points;
            tilted_bbox.scale.x = 0.3;
            tilted_bbox.color.a = 1.0;
            tilted_bbox.pose.orientation.w=1.0;
            switch (classified_obj.type){
                case NOINTEREST:    //WHITE
                    tilted_bbox.color.r = velocity_marker.color.r = 1.0;
                    tilted_bbox.color.g = velocity_marker.color.g = 1.0;
                    tilted_bbox.color.b = velocity_marker.color.b = 1.0;
                    break;
                case APPROACHING:   //YELLOW
                    tilted_bbox.color.r = velocity_marker.color.r = 1.0;
                    tilted_bbox.color.g = velocity_marker.color.g = 1.0;
                    tilted_bbox.color.b = velocity_marker.color.b = 0.5;
                    break;
                case STATIC:       //BLUE
                    tilted_bbox.color.r = velocity_marker.color.r = 0.5;
                    tilted_bbox.color.g = velocity_marker.color.g = 0.5;
                    tilted_bbox.color.b = velocity_marker.color.b = 1.0;
                    break;
                case FRONTOBJ:      //RED
                    tilted_bbox.color.r = velocity_marker.color.r = 1.0;
                    tilted_bbox.color.g = velocity_marker.color.g = 0.5;
                    tilted_bbox.color.b = velocity_marker.color.b = 0.5;
                    break;
                case OPPOSITE:      //GREEN
                    tilted_bbox.color.r = velocity_marker.color.r = 0.5;
                    tilted_bbox.color.g = velocity_marker.color.g = 1.0;
                    tilted_bbox.color.b = velocity_marker.color.b = 0.5;
                    break;
                default:
                    break;
                }

            classified_objs.objlist.push_back(classified_obj);
            text_markers.markers.push_back(text_marker);
            velocity_markers.markers.push_back(velocity_marker);
            velocity_compare_markers.markers.push_back(velocity_marker2);
            tilted_bboxes.markers.push_back(ul_point);
            tilted_bboxes.markers.push_back(tilted_bbox);
        }

    }

    if (tracked_boxes.raw_bounding_boxes.size() ==0){
        // ROS_WARN_STREAM("There is no detected_bboxes");
        classified_objs.raw_objlist.clear();
    }
    else{
        for (int rawobjNum = 0; rawobjNum < tracked_boxes.raw_bounding_boxes.size(); rawobjNum++){
            obj_msgs::Obj raw_object = ConvertBoxtoObj(tracked_boxes.raw_bounding_boxes[rawobjNum]);
            classified_objs.raw_objlist.push_back(raw_object);

            visualization_msgs::Marker raw_bbox;
            raw_bbox.header.frame_id = tracked_boxes.header.frame_id;
            raw_bbox.header.stamp = ros::Time::now();
            raw_bbox.ns = "Detected Boundary";
            raw_bbox.id = 5000+rawobjNum;
            raw_bbox.type = visualization_msgs::Marker::CUBE;
            raw_bbox.action = visualization_msgs::Marker::ADD;
            raw_bbox.lifetime = ros::Duration(0.1);

            raw_bbox.pose.orientation.w = 1.0;
            raw_bbox.pose.position.x = (raw_object.ul.x + raw_object.bl.x)/2.0;
            raw_bbox.pose.position.y = (raw_object.ul.y + raw_object.ur.y)/2.0;
            raw_bbox.pose.position.z = -1.0;
            raw_bbox.scale.x = raw_object.ul.x - raw_object.bl.x;
            raw_bbox.scale.y = raw_object.ul.y - raw_object.ur.y;
            raw_bbox.scale.z = 2.0;

            raw_bbox.color.r = 1.0;
            raw_bbox.color.g = 1.0;
            raw_bbox.color.b = 1.0;
            raw_bbox.color.a = 0.5;
            raw_bboxes.markers.push_back(raw_bbox);
        }
    }

    obj_info_pub.publish(classified_objs);
    obj_id_marker_pub.publish(text_markers);
    obj_velocity_marker_pub.publish(velocity_markers);
    obj_velocity_compare_pub.publish(velocity_compare_markers);
    obj_boundary_marker_pub.publish(tilted_bboxes);
    obj_rawdetect_pub.publish(raw_bboxes);

    ////////////////////
    //// EGO MARKER ////
    ////////////////////
    if(true){
        visualization_msgs::MarkerArray ego_markers;
        visualization_msgs::Marker ego_box;
        visualization_msgs::Marker ego_velocity;
        ego_box.header.frame_id = ego_velocity.header.frame_id = tracked_boxes.header.frame_id;
        ego_box.header.stamp = ego_velocity.header.stamp = ros::Time::now();
        ego_box.color.r = ego_velocity.color.r = 0.9;
        ego_box.color.g = ego_velocity.color.g = 0.9;
        ego_box.color.b = ego_velocity.color.b = 0.9;
        ego_box.color.a = ego_velocity.color.a = 0.8;

        ego_box.id = 123123;
        ego_box.type = visualization_msgs::Marker::CUBE;
        ego_box.action = visualization_msgs::Marker::ADD;
        ego_box.lifetime = ego_velocity.lifetime = ros::Duration(0.1);

        ego_box.pose.orientation.w = 1.0;
        ego_box.pose.position.x = 0;
        ego_box.pose.position.y = 0;
        ego_box.pose.position.z = -1.0;
        ego_box.scale.x = 5.0;
        ego_box.scale.y = 1.8;
        ego_box.scale.z = 2.0;

        ego_velocity.id =124124;
        ego_velocity.type = visualization_msgs::Marker::ARROW;
        ego_velocity.action = visualization_msgs::Marker::ADD;
        ego_velocity.pose.orientation.w = 1.0;
        ego_velocity.points.resize(2);
        ego_velocity.points[0].x = 0;
        ego_velocity.points[0].y = 0;
        ego_velocity.points[0].z = 0;
        ego_velocity.points[1].x = ego_current_speed;
        ego_velocity.points[1].y = 0;
        ego_velocity.points[1].z = 0;
        ego_velocity.scale.x = 2.0;
        ego_velocity.scale.y = 2.0;
        ego_velocity.scale.z = 0;

        ego_markers.markers.push_back(ego_box);
        ego_markers.markers.push_back(ego_velocity);
        ego_markers_pub.publish(ego_markers);
    }
}


obj_msgs::Obj ObjClassiferLidar::ConvertBoxtoObj(obj_msgs::ObjTrackBox &bbox){
    obj_msgs::Obj object;
    object.id = bbox.id;
    object.ul.x = bbox.xmax; - egoLength/2.0;
    object.ul.y = bbox.ymax; - egoWidth/2.0;
    object.ul.z = 0;

    object.ur.x = bbox.xmax; - egoLength/2.0;
    object.ur.y = bbox.ymin; + egoWidth/2.0;
    object.ur.z = 0;

    object.bl.x = bbox.xmin; + egoLength/2.0;
    object.bl.y = bbox.ymax; - egoWidth/2.0;
    object.bl.z = 0;

    object.br.x = bbox.xmin; + egoLength/2.0;
    object.br.y = bbox.ymin; + egoWidth/2.0;
    object.br.z = 0;

    object.pose.position.x = (bbox.xmax + bbox.xmin)/2.0;
    object.pose.position.y = (bbox.ymax + bbox.ymin)/2.0;
    object.pose.position.z = 0;

    float object_angle = atan2(object.pose.position.y, object.pose.position.x);
    // float rotate_magnitude = abs(sqrt(pow(object.pose.position.x,2) + pow(object.pose.position.y,2)) *2* sin(diff_yaw/2.0));
    // float rotate_magnitude = abs(sqrt(pow(object.pose.position.x,2) + pow(object.pose.position.y,2))*cos(diff_yaw/2.))*diff_yaw/0.12;
    // float rotate_magnitude = abs(sqrt(pow(object.pose.position.x,2) + pow(object.pose.position.y,2))*cos(diff_yaw))*diff_yaw/0.12;
    // float rotate_magnitude = sqrt(pow(object.pose.position.x,2) + pow(object.pose.position.y,2))*tan(diff_yaw);
    // float rotate_magnitude = abs(sqrt(pow(object.pose.position.x,2) + pow(object.pose.position.y,2)))/2.0*(sin(diff_yaw)/abs(sin(diff_yaw/2.0)))*diff_yaw/0.1;
    float translate_x_factor = ego_current_speed;
    float translate_y_factor = 0;
    // float rotate_x_factor = -rotate_magnitude*sin(object_angle-diff_yaw/2.0);
    // float rotate_y_factor = +rotate_magnitude*cos(object_angle-diff_yaw/2.0);
    // float rotate_x_factor = -rotate_magnitude*cos(object_angle)*diff_yaw/0.1;
    // float rotate_y_factor = -rotate_magnitude*sin(object_angle)*diff_yaw/0.1;
    float previous_x = object.pose.position.x*cos(diff_yaw) - object.pose.position.y *sin(diff_yaw);
    float previous_y = object.pose.position.x*sin(diff_yaw) + object.pose.position.y *cos(diff_yaw);
    float rotate_x_factor = previous_x - object.pose.position.x;
    float rotate_y_factor = previous_y - object.pose.position.y;

    object.velocity.x = bbox.xdelta + rotate_x_factor + translate_x_factor;    //change from local to absolute speed
    object.velocity.y = bbox.ydelta + rotate_y_factor + translate_y_factor;
    object.velocity.z = 0 ;
    object.velocity_compare.x = bbox.xdelta + translate_x_factor;    //change from local to absolute speed
    object.velocity_compare.y = bbox.ydelta + translate_y_factor;
    object.velocity_compare.z = 0;

    // std::cout<< ego_current_speed<<std::endl;
    // std::cout<< sqrt(pow(object.pose.position.x,2) + pow(object.pose.position.y,2))*diff_yaw<<std::endl;
    // if(object.id != 0){
    //     std::cout<<"id : "<<object.id<<std::endl;
    //     std::cout<<" -> object_angle : "<<(object_angle)*180/M_PI<<std::endl;
    //     std::cout<<" -> magnitude : "<<rotate_magnitude<<std::endl;
    //     std::cout<<"Translation"<<std::endl;
    //     std::cout<<" -> x_factor : "<<translate_x_factor<<std::endl;
    //     std::cout<<" -> y_factor : "<<translate_y_factor<<std::endl;
    //     std::cout<<"Rotation"<<std::endl;
    //     std::cout<<" -> x_factor : "<<rotate_x_factor<<std::endl;
    //     std::cout<<" -> y_factor : "<<rotate_y_factor<<std::endl;
    //     std::cout<<diff_yaw*180.0/M_PI << std::endl;
    //     std::cout << tan(diff_yaw) << std::endl;
    // }

    return object;
}

int ObjClassiferLidar::DefineObjType(obj_msgs::Obj &object){
    ego_link_type = ego_link_type_temp;

    std::vector<int> local_link_indices;
    Kdtree(link_pnts, object.pose.position.x, object.pose.position.y, local_link_indices);
    pcl::PointXYZI target_link_point = link_pnts.points[local_link_indices[0]];
    double target_link_diff_yaw = getNeighborPntsTheta(link_pnts, target_link_point.x, target_link_point.y);
    int target_link_type = target_link_point.intensity;



    double tilted_target_poseX = (object.pose.position.x - (object.ul.x - object.bl.x))*cos(-ego_link_diff_theta) - object.pose.position.y*sin(-ego_link_diff_theta);
    double tilted_target_poseY = (object.pose.position.x - (object.ul.x - object.bl.x))*sin(-ego_link_diff_theta) + object.pose.position.y*cos(-ego_link_diff_theta);
    double local_refer_poseX = (-dist2egotail)*cos(-ego_link_diff_theta);
    double local_refer_pose_backX = (-4*dist2egotail)*cos(-ego_link_diff_theta);
    double local_refer_poseY = (-dist2egotail)*sin(-ego_link_diff_theta);

    double target_ub_size = object.ul.x-object.bl.x;
    double target_lr_size = object.bl.y-object.br.y;
    double box_xdelta = object.ul.x - object.bl.x;
    double box_ydelta = object.ul.y - object.bl.y;
    double box_yaw = atan2(box_ydelta, box_xdelta);
    std::vector<geometry_msgs::Point> tiltedBox_points = TilteBox(object, -(box_yaw - target_link_diff_yaw));
    if (abs(-(box_yaw - target_link_diff_yaw) - M_PI/2.0) < abs(-(box_yaw - target_link_diff_yaw))){
      object.ur = tiltedBox_points[0];
      object.br = tiltedBox_points[1];
      object.bl = tiltedBox_points[2];
      object.ul = tiltedBox_points[3];
    }
    else{
      object.ul = tiltedBox_points[0];
      object.ur = tiltedBox_points[1];
      object.br = tiltedBox_points[2];
      object.bl = tiltedBox_points[3];
    }

    if(use_velocity_fitting){
        double vector_theta = atan2(object.velocity.y, object.velocity.x);
        double vector_x = object.velocity.x;
        double vector_y = object.velocity.y;
        //BACK
        if (object.pose.position.x < 0){
            if( (abs(vector_theta - target_link_diff_yaw) < velocity_fitting_low_thd*M_PI/180.0)){
                object.velocity.x = vector_x * cos(-(vector_theta - target_link_diff_yaw)) - vector_y * sin(-(vector_theta - target_link_diff_yaw));
                object.velocity.y = vector_x * sin(-(vector_theta - target_link_diff_yaw)) + vector_y * cos(-(vector_theta - target_link_diff_yaw));
            }
            if( (abs(vector_theta - target_link_diff_yaw) > M_PI - velocity_fitting_low_thd*M_PI/180.0)){
                object.velocity.x = vector_x * cos(-(vector_theta - target_link_diff_yaw - M_PI)) - vector_y * sin(-(vector_theta - target_link_diff_yaw - M_PI));
                object.velocity.y = vector_x * sin(-(vector_theta - target_link_diff_yaw - M_PI)) + vector_y * cos(-(vector_theta - target_link_diff_yaw - M_PI));
            }
        }
        if (object.pose.position.y * (vector_theta - target_link_diff_yaw) >= 0 ){
            ROS_INFO_STREAM(abs(vector_theta-target_link_diff_yaw)*180.0/M_PI);
            if( (abs(vector_theta - target_link_diff_yaw) < velocity_fitting_high_thd*M_PI/180.0) ){
                object.velocity.x = vector_x * cos(-(vector_theta - target_link_diff_yaw)) - vector_y * sin(-(vector_theta - target_link_diff_yaw));
                object.velocity.y = vector_x * sin(-(vector_theta - target_link_diff_yaw)) + vector_y * cos(-(vector_theta - target_link_diff_yaw));
            }
            if((abs(vector_theta - target_link_diff_yaw) > M_PI - velocity_fitting_high_thd*M_PI/180.0) ){
                object.velocity.x = vector_x * cos(-(vector_theta - target_link_diff_yaw - M_PI)) - vector_y * sin(-(vector_theta - target_link_diff_yaw - M_PI));
                object.velocity.y = vector_x * sin(-(vector_theta - target_link_diff_yaw - M_PI)) + vector_y * cos(-(vector_theta - target_link_diff_yaw - M_PI));
            }
        }
        else{
            if( (abs(vector_theta - target_link_diff_yaw) < velocity_fitting_low_thd*M_PI/180.0)){
                object.velocity.x = vector_x * cos(-(vector_theta - target_link_diff_yaw)) - vector_y * sin(-(vector_theta - target_link_diff_yaw));
                object.velocity.y = vector_x * sin(-(vector_theta - target_link_diff_yaw)) + vector_y * cos(-(vector_theta - target_link_diff_yaw));
            }
            if( (abs(vector_theta - target_link_diff_yaw) > M_PI - velocity_fitting_low_thd*M_PI/180.0)){
                object.velocity.x = vector_x * cos(-(vector_theta - target_link_diff_yaw - M_PI)) - vector_y * sin(-(vector_theta - target_link_diff_yaw - M_PI));
                object.velocity.y = vector_x * sin(-(vector_theta - target_link_diff_yaw - M_PI)) + vector_y * cos(-(vector_theta - target_link_diff_yaw - M_PI));
            }
        }
    }

    // //TERNEL
    // if(ego_link_type != CLOCK_WISE_TERNEL_1 ||ego_link_type != CLOCK_WISE_TERNEL_2 || ego_link_type != COUNTER_CLOCK_WISE_TERNEL_1 || ego_link_type != COUNTER_CLOCK_WISE_TERNEL_2 ){
    //     //EGO: NORMAL ROAD -> make target normal case
    //     if(target_link_type == CLOCK_WISE_TERNEL_1 || target_link_type == CLOCK_WISE_TERNEL_2){
    //         target_link_type = CLOCK_WISE;
    //     }
    //     if(target_link_type == COUNTER_CLOCK_WISE_TERNEL_1 || target_link_type == COUNTER_CLOCK_WISE_TERNEL_2){
    //         target_link_type = COUNTER_CLOCK_WISE;
    //     }
    // }
    // else{
    //     // EGO in TERNEL
    //     if(ego_link_type == CLOCK_WISE_TERNEL_1 || ego_link_type == CLOCK_WISE_TERNEL_2){
    //         if(target_link_type == COUNTER_CLOCK_WISE || target_link_type == COUNTER_CLOCK_WISE_TERNEL_1 || target_link_type == COUNTER_CLOCK_WISE_TERNEL_2){
    //             return OPPOSITE;
    //         }
    //     }
    //     else{
    //         if(target_link_type == CLOCK_WISE || target_link_type == CLOCK_WISE_TERNEL_1 || target_link_type == CLOCK_WISE_TERNEL_2){
    //             return OPPOSITE;
    //         }
    //     }
    //
    //     if(target_link_type == CLOCK_WISE_TERNEL_1 || target_link_type == CLOCK_WISE_TERNEL_2 || target_link_type == COUNTER_CLOCK_WISE_TERNEL_1 || target_link_type == COUNTER_CLOCK_WISE_TERNEL_2){
    //         if(ego_link_type != target_link_type) {return NOINTEREST;}
    //         else{
    //             if(tilted_target_poseX < 0){return NOINTEREST;}
    //             if(is_static(object)){return STATIC;}
    //             else{return FRONTOBJ;}
    //         }
    //     }
    //     else{
    //         if(tilted_target_poseX < 0){return NOINTEREST;}
    //         if(is_static(object)){return STATIC;}
    //         else{return FRONTOBJ;}
    //     }
    // }

    //SLOPE
    if (target_link_type == SLOPE || ego_link_type == SLOPE){
        if(ego_link_type == target_link_type){
            //make normal case
            target_link_type = ego_link_type = INTERSECTION;
        }
        else{
            double dist_object = sqrt(object.pose.position.x*object.pose.position.x + object.pose.position.y*object.pose.position.y);
            double size_thd = 6.0;
            double dist_thd = 30.0;
            if(dist_object > dist_thd){return NOINTEREST;}
            if(target_ub_size > size_thd || target_lr_size > size_thd){return NOINTEREST;}
            else{target_link_type = INTERSECTION;}
        }
    }

    //NORMAL case
    if(ego_link_type != INTERSECTION){
        if(!((ego_link_type == target_link_type)||(target_link_type == INTERSECTION))){
            if(is_static(object)){return OPPOSITE;}
            if(object.velocity.y < 0){return OPPOSITE;}
            return OPPOSITE;
        }
    }

    if( (abs(tilted_target_poseY)<abs(local_refer_poseY+1)  && (tilted_target_poseX <  local_refer_poseX)) || (abs(object.pose.position.y) <= 1.0  && (tilted_target_poseX <  local_refer_poseX)) ){
        return NOINTEREST;
    }

    if(is_static(object)){
        if(tilted_target_poseX >= local_refer_pose_backX){return STATIC;}
        else{return NOINTEREST;}
    }
    else{
        if(tilted_target_poseX >=  local_refer_poseX){
            if(object.velocity.x < 0){return APPROACHING;}
            else{return FRONTOBJ;}
        }
        else{
            if(object.velocity.x >= 0){return APPROACHING;}
            else{return NOINTEREST;}
        }
    }



}

std::vector<geometry_msgs::Point> ObjClassiferLidar::TilteBox(obj_msgs::Obj &object, double tilt_theta){
    Eigen::Matrix2f tilt_matrix;
    std::vector<geometry_msgs::Point> boundary_points;
    if (abs(tilt_theta - M_PI/2.0) < abs(tilt_theta)){
      tilt_theta -= M_PI/2.0;
    }
    boundary_points.push_back(object.ul);
    boundary_points.push_back(object.ur);
    boundary_points.push_back(object.br);
    boundary_points.push_back(object.bl);

    //reverse rotate
    tilt_matrix <<  cos(tilt_theta), -sin(tilt_theta),
                    sin(tilt_theta),  cos(tilt_theta);

    std::vector<geometry_msgs::Point> tilted_points;
    for (int i = 0 ; i < 4; i++){
        geometry_msgs::Point tilted_point;
        tilted_point.x =  tilt_matrix(0,0) * (boundary_points[i].x - object.pose.position.x) + tilt_matrix(0,1) * (boundary_points[i].y - object.pose.position.y);
        tilted_point.y =  tilt_matrix(1,0) * (boundary_points[i].x - object.pose.position.x) + tilt_matrix(1,1) * (boundary_points[i].y - object.pose.position.y);
        tilted_point.z =  0.0;
        tilted_point.x += object.pose.position.x;
        tilted_point.y += object.pose.position.y;
        tilted_points.push_back(tilted_point);
    }
    return tilted_points;
}

bool ObjClassiferLidar::is_static(obj_msgs::Obj &object){
    if (object.velocity.x * object.velocity.x + object.velocity.y * object.velocity.y < staticThd*staticThd){
        return true;
    }
    else{
        return false;
    }
}

double ObjClassiferLidar::getNeighborPntsTheta(pcl::PointCloud<pcl::PointXYZI> &src_pc, float x_search, float y_search){
    std::vector<int> searched_indices;

    Kdtree(src_pc, x_search, y_search, searched_indices);

    int xmax_idx = searched_indices[0];
    int xmin_idx = searched_indices[0];
    int ymax_idx = searched_indices[0];
    int ymin_idx = searched_indices[0];
    for (int pntNum = 1; pntNum < searched_indices.size(); pntNum++){
        if(src_pc.points[searched_indices[pntNum]].x >= src_pc.points[xmax_idx].x){
            xmax_idx = searched_indices[pntNum];
        }
        if(src_pc.points[searched_indices[pntNum]].x < src_pc.points[xmin_idx].x){
            xmin_idx = searched_indices[pntNum];
        }
        if(src_pc.points[searched_indices[pntNum]].y >= src_pc.points[ymax_idx].y){
            ymax_idx = searched_indices[pntNum];
        }
        if(src_pc.points[searched_indices[pntNum]].y < src_pc.points[ymin_idx].y){
            ymin_idx = searched_indices[pntNum];
        }
    }
    double xdelta_X = src_pc.points[xmax_idx].x - src_pc.points[xmin_idx].x;
    double ydelta_X = src_pc.points[xmax_idx].y - src_pc.points[xmin_idx].y;
    double xdelta_Y = src_pc.points[ymax_idx].x - src_pc.points[ymin_idx].x;
    double ydelta_Y = src_pc.points[ymax_idx].y - src_pc.points[ymin_idx].y;
    double theta;
    if (xdelta_X >=ydelta_Y){ theta = atan2(ydelta_X, xdelta_X); }
    else {theta = atan2(ydelta_Y, xdelta_Y);}
    // else {theta = atan2(ydelta_Y, xdelta_Y) - M_PI/2;}

    return theta;
}

void ObjClassiferLidar::Kdtree(pcl::PointCloud<pcl::PointXYZI> &src_pc, float x_search, float y_search, std::vector<int> &searched_indices){
    pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;
    std::vector<int> k_indices;
    std::vector<float> k_distances;
    pcl::PointXYZI k_searchPoint;
    k_searchPoint.x = x_search;
    k_searchPoint.y = y_search;
    k_searchPoint.z = 0;
    pcl::PointCloud<pcl::PointXYZI>::Ptr input_pc(new pcl::PointCloud<pcl::PointXYZI>);
    *input_pc = src_pc;
    kdtree.setInputCloud(input_pc);
    kdtree.nearestKSearch(k_searchPoint, 3, k_indices, k_distances);
    searched_indices = k_indices;
}
