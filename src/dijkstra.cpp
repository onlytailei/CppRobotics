/*************************************************************************
	> File Name: a_star.cpp
	> Author: TAI Lei
	> Mail: ltai@ust.hk
	> Created Time: Sat Jul 20 12:38:43 2019
 ************************************************************************/

#include<iostream>
#include<cmath>
#include<limits>
#include<queue>
#include<vector>
#include<opencv2/opencv.hpp>
#include<opencv2/core/core.hpp>
#include<opencv2/highgui/highgui.hpp>
using namespace std;


class Node{
public:
  int x;
  int y;
  float cost;
  Node* p_node;

  Node(int x_, int y_, float cost_, Node* p_node_=NULL):x(x_), y(y_),cost(cost_), p_node(p_node_){};
};


std::vector<std::vector<float> > calc_final_path(Node * goal, float reso, cv::Mat& img, float img_reso){
  std::vector<float> rx;
  std::vector<float> ry;
  Node* node = goal;
  while (node->p_node != NULL){
    node = node->p_node;
    rx.push_back(node->x * reso);
    ry.push_back(node->y * reso);
    cv::rectangle(img,
        cv::Point(node->x*img_reso+1, node->y*img_reso+1),
        cv::Point((node->x+1)*img_reso, (node->y+1)*img_reso),
        cv::Scalar(255, 0, 0), -1);
  }
  return {rx, ry};
}


std::vector<std::vector<int> > calc_obstacle_map(
    std::vector<int> ox, std::vector<int> oy,
    const int min_ox, const int max_ox,
    const int min_oy, const int max_oy,
    float reso, float vr,
    cv::Mat& img, int img_reso){

  int xwidth = max_ox-min_ox;
  int ywidth = max_oy-min_oy;

  std::vector<std::vector<int> > obmap(ywidth, vector<int>(xwidth, 0));

  for(int i=0; i<xwidth; i++){
    int x = i + min_ox;
    for(int j=0; j<ywidth; j++){
      int y = j + min_oy;
      for(int k=0; k<ox.size(); k++){
        float d = std::sqrt(std::pow((ox[k]-x), 2)+std::pow((oy[k]-y), 2));
        if (d <= vr/reso){
          obmap[i][j] = 1;
          cv::rectangle(img,
                        cv::Point(i*img_reso+1, j*img_reso+1),
                        cv::Point((i+1)*img_reso, (j+1)*img_reso),
                        cv::Scalar(0, 0, 0), -1);
          break;
        }
      }
    }
  }
  return obmap;
}


bool verify_node(Node* node,
                 vector<vector<int> > obmap,
                 int min_ox, int max_ox,
                 int min_oy, int max_oy){
  if (node->x < min_ox || node->y < min_oy || node->x >= max_ox || node->y >= max_oy){
    return false;
  }

  if (obmap[node->x-min_ox][node->y-min_oy]) return false;

  return true;
}


float calc_heristic(Node n1, Node n2, float w=1.0){
  return w * std::sqrt(std::pow(n1.x-n2.x, 2)+std::pow(n1.y-n2.y, 2));
}

std::vector<Node> get_motion_model(){
  return {Node(1,   0,  1),
          Node(0,   1,  1),
          Node(-1,   0,  1),
          Node(0,   -1,  1),
          Node(-1,   -1,  std::sqrt(2)),
          Node(-1,   1,  std::sqrt(2)),
          Node(1,   -1,  std::sqrt(2)),
          Node(1,    1,  std::sqrt(2))};
}

void dijkstra_star_planning(float sx, float sy,
                     float gx, float gy,
                     vector<float> ox_, vector<float> oy_,
                     float reso, float rr){

  Node* nstart = new Node((int)std::round(sx/reso), (int)std::round(sy/reso), 0.0);
  Node* ngoal = new Node((int)std::round(gx/reso), (int)std::round(gy/reso), 0.0);


  vector<int> ox;
  vector<int> oy;

  int min_ox = std::numeric_limits<int>::max();
  int max_ox = std::numeric_limits<int>::min();
  int min_oy = std::numeric_limits<int>::max();
  int max_oy = std::numeric_limits<int>::min();


  for(float iox:ox_){
      int map_x = (int)std::round(iox*1.0/reso);
      ox.push_back(map_x);
      min_ox = std::min(map_x, min_ox);
      max_ox = std::max(map_x, max_ox);
  }

  for(float ioy:oy_){
      int map_y = (int)std::round(ioy*1.0/reso);
      oy.push_back(map_y);
      min_oy = std::min(map_y, min_oy);
      max_oy = std::max(map_y, max_oy);
  }

  int xwidth = max_ox-min_ox;
  int ywidth = max_oy-min_oy;

  //visualization
  cv::namedWindow("astar", cv::WINDOW_NORMAL);
  int count = 0;
  int img_reso = 5;
  cv::Mat bg(img_reso*xwidth,
             img_reso*ywidth,
             CV_8UC3,
             cv::Scalar(255,255,255));

    cv::rectangle(bg,
                  cv::Point(nstart->x*img_reso+1, nstart->y*img_reso+1),
                  cv::Point((nstart->x+1)*img_reso, (nstart->y+1)*img_reso),
                  cv::Scalar(255, 0, 0), -1);
    cv::rectangle(bg,
                  cv::Point(ngoal->x*img_reso+1, ngoal->y*img_reso+1),
                  cv::Point((ngoal->x+1)*img_reso, (ngoal->y+1)*img_reso),
                  cv::Scalar(0, 0, 255), -1);

  std::vector<std::vector<int> > visit_map(xwidth, vector<int>(ywidth, 0));


  std::vector<std::vector<int> > obmap = calc_obstacle_map(
                                                  ox, oy,
                                                  min_ox, max_ox,
                                                  min_oy, max_oy,
                                                  reso, rr,
                                                  bg, img_reso);

  // NOTE: d_ary_heap should be a better choice here
  auto cmp = [](const Node* left, const Node* right){return left->cost > right->cost;};
  std::priority_queue<Node*, std::vector<Node*>, decltype(cmp)> pq(cmp);

  pq.push(nstart);
  std::vector<Node> motion = get_motion_model();

  while (true){
    Node * node = pq.top();

    if (visit_map[node->x-min_ox][node->y-min_oy] == 1){
      pq.pop();
      delete node;
      continue;
    }else{
      pq.pop();
      visit_map[node->x-min_ox][node->y-min_oy] = 1;
    }

    if (node->x == ngoal->x && node->y==ngoal->y){
      ngoal->cost = node->cost;
      ngoal->p_node = node;
      break;
    }

    for(int i=0; i<motion.size(); i++){
      Node * new_node = new Node(
        node->x + motion[i].x,
        node->y + motion[i].y,
        node->cost + motion[i].cost,
        node);

      if (!verify_node(new_node, obmap, min_ox, max_ox, min_oy, max_oy)){
        delete new_node;
        continue;
      }

      if (visit_map[new_node->x-min_ox][new_node->y-min_oy]){
        delete new_node;
        continue;
      }

      cv::rectangle(bg,
                    cv::Point(new_node->x*img_reso+1, new_node->y*img_reso+1),
                    cv::Point((new_node->x+1)*img_reso, (new_node->y+1)*img_reso),
                    cv::Scalar(0, 255, 0));

      // std::string int_count = std::to_string(count);
      // cv::imwrite("./pngs/"+std::string(5-int_count.length(), '0').append(int_count)+".png", bg);
      count++;
      cv::imshow("dijkstra", bg);
      cv::waitKey(5);

      pq.push(new_node);
    }
  }

  calc_final_path(ngoal, reso, bg, img_reso);
  delete ngoal;
  delete nstart;

  // std::string int_count = std::to_string(count);
  // cv::imwrite("./pngs/"+std::string(5-int_count.length(), '0').append(int_count)+".png", bg);
  cv::imshow("dijkstra", bg);
  cv::waitKey(5);
};


int main(){
  float sx = 10.0;
  float sy = 10.0;
  float gx = 50.0;
  float gy = 50.0;

  float grid_size = 1.0;
  float robot_size = 1.0;

  vector<float> ox;
  vector<float> oy;

  // add edges
  for(float i=0; i<60; i++){
    ox.push_back(i);
    oy.push_back(60.0);
  }
  for(float i=0; i<60; i++){
    ox.push_back(60.0);
    oy.push_back(i);
  }
  for(float i=0; i<61; i++){
    ox.push_back(i);
    oy.push_back(60.0);
  }
  for(float i=0; i<61; i++){
    ox.push_back(0.0);
    oy.push_back(i);
  }
  for(float i=0; i<40; i++){
    ox.push_back(20.0);
    oy.push_back(i);
  }
  for(float i=0; i<40; i++){
    ox.push_back(40.0);
    oy.push_back(60.0 - i);
  }

  dijkstra_star_planning(sx, sy, gx, gy, ox, oy, grid_size, robot_size);
  return 0;
}
