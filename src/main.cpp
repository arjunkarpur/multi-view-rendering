#include <igl/viewer/Viewer.h>
#include <igl/readObj.h>
#include <iostream>
#include <igl/png/writePNG.h>
#include <unistd.h>

#define PI 3.14159265358979

bool key_down(igl::viewer::Viewer& viewer, unsigned char key, int modifier) {
  if (key == ' ') {
    /*
    double angle_x = (PI/6);
    Eigen::Matrix3f test;
    test << 
      1, 0, 0,
      0, std::cos(angle_x), std::sin(angle_x),
      0, 0-std::sin(angle_x), std::cos(angle_x);
      */

    int x_jumps = 3;
    float x_angles[x_jumps];
    x_angles[0] = PI/6;
    x_angles[1] = 0;
    x_angles[2] = -PI/6;
    int y_jumps = 15;
    Eigen::Matrix3f xRotate;
    Eigen::Matrix3f yRotate;

    Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic> R(1280, 800);
    Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic> G(1280, 800);
    Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic> B(1280, 800);
    Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic> A(1280, 800);

    for (int i = 0; i < x_jumps; i++) {
      xRotate << 
        1, 0, 0,
        0, std::cos(x_angles[i]), std::sin(x_angles[i]),
        0, 0-std::sin(x_angles[i]), std::cos(x_angles[i]);

      for (int j = 0; j < y_jumps; j++) {

        float y_angle = ((2*PI)/float(y_jumps))*float(j);
        yRotate <<
          std::cos(y_angle), 0, -std::sin(y_angle),
          0, 1, 0,
          std::sin(y_angle), 0, std::cos(y_angle);

        Eigen::Quaternionf rot(xRotate*yRotate);
        viewer.core.trackball_angle = rot;
        viewer.draw();

        viewer.core.draw_buffer(viewer.data, viewer.opengl, false, R,G,B,A);
        std::stringstream out_name;
        out_name << "../output/" << i << "_" << j << ".png";
        igl::png::writePNG(R,G,B,A,out_name.str());
      }
    }
  }

    /*
    float x_angles[3];
    x_angles[0] = -(PI/6);
    x_angles[1] = 0;
    x_angles[2] = -(PI/6);
    int y_jumps = 15;
    Eigen::Matrix3f xRotate;
    Eigen::Matrix3f yRotate;
    for (int i = 0; i < sizeof(x_angles); i++) {
      xRotate <<
        1, 0, 0,
        0, std::cos(x_angles[i]), std::sin(x_angles[i]),
        0, 0-std::sin(x_angles[i]), std::cos(x_angles[i]);

      for (int j = 0; j < y_jumps; j++) {
        float y_angle = (PI/(float(y_jumps)))*(float(j));
        yRotate << 
          std::cos(y_angle), 0, 0-std::sin(y_angle),
          0, 1, 0,
          std::sin(y_angle), 0, std::cos(y_angle);

        // Apply camera rotation
        Eigen::Quaternionf rot(xRotate);
        viewer.core.trackball_angle = rot;

        break;
      }
    }
  }
  */
  return false;
}

int main(int argc, char *argv[])
{

  // Load mesh
  std::string name = "../../../data/meshes/healthy/female_asian_3-1_7603068_1.obj";
  Eigen::MatrixXd V;
  Eigen::MatrixXi F;
  igl::readOBJ(name, V, F);

  // Orient so image facing forwards
  double angle_x = -1.57;
  double angle_y = 1.57;
  Eigen::MatrixXd rotation_x(3,3);
  Eigen::MatrixXd rotation_y(3,3);
  rotation_x << 
    1, 0, 0,
    0, std::cos(angle_x), std::sin(angle_x),
    0, 0-std::sin(angle_x), std::cos(angle_x);
  rotation_y << 
    std::cos(angle_y), 0, 0-std::sin(angle_y),
    0, 1, 0,
    std::sin(angle_y), 0, std::cos(angle_y);
  V = V * rotation_x;
  V = V * rotation_y;

    // Plot the mesh
  igl::viewer::Viewer viewer;
  viewer.callback_key_down = &key_down;
  viewer.data.set_mesh(V, F);
  viewer.data.set_face_based(false);
  viewer.data.set_normals(viewer.data.V_normals);
  viewer.launch();
}
