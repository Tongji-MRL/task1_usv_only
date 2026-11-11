#include "robotx_task1_nav/traffic_layer.hpp"
#include <iostream>

class Harness : public robotx_task1_nav::TrafficLayer {
public:
  using nav2_costmap_2d::StaticLayer::incomingMap;
};
void check(bool value,const char *message){if(!value)throw std::runtime_error(message);}
int main(int argc,char **argv){
  rclcpp::init(argc,argv);
  auto node=std::make_shared<nav2_util::LifecycleNode>("traffic_layer_test");
  node->declare_parameter("map_topic","/traffic_layer_test/input");
  node->declare_parameter("track_unknown_space",true);
  node->declare_parameter("use_maximum",false);
  node->declare_parameter("lethal_cost_threshold",100);
  node->declare_parameter("unknown_cost_value",255);
  node->declare_parameter("trinary_costmap",true);
  node->declare_parameter("transform_tolerance",0.);
  auto group=node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  tf2_ros::Buffer tf(node->get_clock());
  for(bool rolling:{false,true}){
    nav2_costmap_2d::LayeredCostmap parent("map",rolling,true);
    parent.resizeMap(10,10,1.,0.,0.);
    auto layer=std::make_shared<Harness>();parent.addPlugin(layer);
    layer->initialize(&parent,rolling?"rolling_prediction":"fixed_prediction",&tf,node,group);
    check(!node->get_parameter("use_maximum").as_bool(),"base static layer setting must remain unchanged");
    auto message=std::make_shared<nav_msgs::msg::OccupancyGrid>();
    message->header.frame_id="map";message->info.resolution=1.;message->info.width=10;message->info.height=10;
    message->info.origin.orientation.w=1.;message->data.assign(100,0);message->data[44]=100;
    auto update=[&](){
      layer->incomingMap(message);
      double a=1e9,b=1e9,c=-1e9,d=-1e9;layer->updateBounds(5.,5.,0.,&a,&b,&c,&d);
      auto master=parent.getCostmap();
      // Reconstruct the preceding static/known-buoy layers for this cycle.
      for(unsigned y=0;y<10;++y)for(unsigned x=0;x<10;++x)master->setCost(x,y,0);
      master->setCost(1,1,254);master->setCost(8,8,254);master->setCost(2,3,255);
      layer->updateCosts(*master,0,0,10,10);
      check(master->getCost(1,1)==254&&master->getCost(8,8)==254,"empty traffic cells must preserve existing obstacles");
      check(master->getCost(2,2)==0,"unoccupied water remains free");
      check(master->getCost(2,3)==255,"empty inference must not turn unknown cells into free water");
    };
    update();check(parent.getCostmap()->getCost(4,4)==254,"new traffic body must be lethal");
    message=std::make_shared<nav_msgs::msg::OccupancyGrid>(*message);
    message->data[44]=0;message->data[46]=100;update();
    check(parent.getCostmap()->getCost(4,4)==0,"old traffic location clears on new complete snapshot");
    check(parent.getCostmap()->getCost(6,4)==254,"moved body appears at its new location");
  }
  node.reset();rclcpp::shutdown();
  std::cout<<"Traffic layer merge and clearing passed for fixed and rolling maps\n";
}
