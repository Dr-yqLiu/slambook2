#include <pangolin/pangolin.h>
#include <Eigen/Core>
#include <unistd.h>

// 本例演示了如何画出一个预先存储的轨迹

using namespace std;
using namespace Eigen;

// path to trajectory file
string trajectory_file = "/Users/liujia/Desktop/CppProgram/slambook2/ch3/examples/trajectory.txt";

void DrawTrajectory(vector<Isometry3d, Eigen::aligned_allocator<Isometry3d>>);

int main(int argc, char **argv)
{

  vector<Isometry3d, Eigen::aligned_allocator<Isometry3d>> poses;
  ifstream fin(trajectory_file);
  if (!fin)
  {
    cout << "cannot find trajectory file at " << trajectory_file << endl;
    return 1;
  }

  while (!fin.eof())
  {
    double time, tx, ty, tz, qx, qy, qz, qw;
    fin >> time >> tx >> ty >> tz >> qx >> qy >> qz >> qw;
    Isometry3d Twr(Quaterniond(qw, qx, qy, qz));
    Twr.pretranslate(Vector3d(tx, ty, tz));
    poses.push_back(Twr);
  }
  cout << "read total " << poses.size() << " pose entries" << endl;

  // draw trajectory in pangolin
  DrawTrajectory(poses);
  return 0;
}

/*******************************************************************************************/
void DrawTrajectory(vector<Isometry3d, Eigen::aligned_allocator<Isometry3d>> poses)
{
  // create pangolin window and plot the trajectory
  // 创建窗口，分辨率1024x768
  pangolin::CreateWindowAndBind("Trajectory Viewer", 1024, 768);

  // 启用深度测试，用于正确处理三维对象的遮挡关系
  glEnable(GL_DEPTH_TEST);

  // 启用并配置混合模式，允许半透明效果（即物体的透明度可以被正确处理）
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

  // 设置摄像机的投影矩阵和视图矩阵
  pangolin::OpenGlRenderState s_cam(
      // 这是一个透视投影矩阵，定义了投影的视角、近平面和远平面等参数
      pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 389, 0.1, 1000),
      // 定义了摄像机的位置和朝向，摄像机从 (0, -0.1, -1.8) 看向原点 (0, 0, 0)，并且定义了 "上" 的方向为 y 轴负方向
      pangolin::ModelViewLookAt(0, -0.1, -1.8, 0, 0, 0, 0.0, -1.0, 0.0));

  // 创建视图对象 d_cam，由摄像机s_cam配置并控制视图d_cam
  pangolin::View &d_cam = pangolin::CreateDisplay()
                              .SetBounds(0.0, 1.0, 0.0, 1.0, -1024.0f / 768.0f)
                              .SetHandler(new pangolin::Handler3D(s_cam));

  // 这是一个渲染循环，只要窗口未被关闭，就会不断地运行
  while (pangolin::ShouldQuit() == false)
  {
    // 清除颜色和深度缓冲区，准备绘制新的帧
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // 激活摄像机的视图矩阵，准备绘制内容
    d_cam.Activate(s_cam);

    // 设置背景颜色为白色
    glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

    // 设置线条宽度为2个像素
    glLineWidth(2);
    for (size_t i = 0; i < poses.size(); i++)
    {
      // 画每个位姿的三个坐标轴
      Vector3d Ow = poses[i].translation();

      // 通过运算符重载，相当于计算出了连体坐标轴在全局坐标系下的方位，并且模长缩小到0.1
      Vector3d Xw = poses[i] * (0.1 * Vector3d(1, 0, 0));
      Vector3d Yw = poses[i] * (0.1 * Vector3d(0, 1, 0));
      Vector3d Zw = poses[i] * (0.1 * Vector3d(0, 0, 1));

      // 绘制从原点 Ow 到 X、Y、Z 轴方向的小线段，分别用红、绿、蓝颜色表示
      glBegin(GL_LINES);
      glColor3f(1.0, 0.0, 0.0);
      glVertex3d(Ow[0], Ow[1], Ow[2]);
      glVertex3d(Xw[0], Xw[1], Xw[2]);
      glColor3f(0.0, 1.0, 0.0);
      glVertex3d(Ow[0], Ow[1], Ow[2]);
      glVertex3d(Yw[0], Yw[1], Yw[2]);
      glColor3f(0.0, 0.0, 1.0);
      glVertex3d(Ow[0], Ow[1], Ow[2]);
      glVertex3d(Zw[0], Zw[1], Zw[2]);
      glEnd();
    }
    // 画出连线
    for (size_t i = 0; i < poses.size(); i++)
    {
      // 每两个相邻位姿的连线，用黑色线条来表示轨迹路径
      glColor3f(0.0, 0.0, 0.0);
      glBegin(GL_LINES);
      auto p1 = poses[i], p2 = poses[i + 1];
      glVertex3d(p1.translation()[0], p1.translation()[1], p1.translation()[2]);
      glVertex3d(p2.translation()[0], p2.translation()[1], p2.translation()[2]);
      glEnd();
    }

    // 告诉 Pangolin 当前帧绘制完毕，准备绘制下一帧
    pangolin::FinishFrame();

    // 暂停5毫秒，以控制帧率
    usleep(5000); // sleep 5 ms
  }
}
