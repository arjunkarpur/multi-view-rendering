#include <igl/viewer/Viewer.h>
#include <igl/readObj.h>
#include <iostream>
#include <igl/png/writePNG.h>
#include <unistd.h>
#include <vector>
#include <fstream>

#define PI 3.14159265358979

std::vector<std::string> *mesh_filepaths;
std::string output_dir;

void captureImages(igl::viewer::Viewer& viewer) {

  // Initialize png buffers
  int width = 1280;
  int height = 800;
  Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic> R(width, height);
  Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic> G(width, height);
  Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic> B(width, height);
  Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic> A(width, height);

  // Initialize vars needed for camera rotations
  int x_jumps = 3;
  float x_angles[x_jumps];
  x_angles[0] = PI/6;
  x_angles[1] = 0;
  x_angles[2] = -PI/6;
  int y_jumps = 15;
  Eigen::Matrix3f xRotate;
  Eigen::Matrix3f yRotate;

  for (int i = 0; i < mesh_filepaths->size(); i++) {

    // Load mesh
    std::string mesh_fp = mesh_filepaths->at(i);
    Eigen::MatrixXd V;
    Eigen::MatrixXi F;
    igl::readOBJ(mesh_fp, V, F);

    // Create output directory
    std::stringstream new_dir;
    new_dir << output_dir << "/" << mesh_fp.substr(0, mesh_fp.size()-4);
    std::stringstream new_dir_cmd;
    new_dir_cmd << "mkdir " << new_dir.str();
    const char* cmd = (new_dir_cmd.str()).c_str();
    system(cmd);

    // Orient so image facing forwards
    //   TODO: hardcoded. need to read from metadata file
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

    // Draw mesh to viewer
    viewer.data.set_mesh(V,F);
    viewer.data.set_face_based(false);
    viewer.data.set_normals(viewer.data.V_normals);
    viewer.core.align_camera_center(V,F);
    viewer.draw();


    for (int j = 0; j < x_jumps; j++) {
      xRotate << 
        1, 0, 0,
        0, std::cos(x_angles[j]), std::sin(x_angles[j]),
        0, 0-std::sin(x_angles[j]), std::cos(x_angles[j]);

      for (int k = 0; k < y_jumps; k++) {

        float y_angle = ((2*PI)/float(y_jumps))*float(k);
        yRotate <<
          std::cos(y_angle), 0, -std::sin(y_angle),
          0, 1, 0,
          std::sin(y_angle), 0, std::cos(y_angle);

        Eigen::Quaternionf rot(xRotate*yRotate);
        viewer.core.trackball_angle = rot;
        viewer.draw();

        viewer.core.draw_buffer(viewer.data, viewer.opengl, false, R,G,B,A);
        std::stringstream out_name;
        out_name << output_dir << "/" << j << "_" << k << ".png";
        igl::png::writePNG(R,G,B,A,out_name.str());
      }
    }
  }
}

bool key_down(igl::viewer::Viewer& viewer, unsigned char key, int modifier) {
  if (key == ' ') {
    captureImages(viewer);
  }
  return false;
}
    
/*
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
    viewer.data.set_mesh(V,F);
    viewer.data.set_face_based(false);
    viewer.data.set_normals(viewer.data.V_normals);
    viewer.core.align_camera_center(V,F);
    viewer.draw();

    ---------

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
  return false;
}
*/

std::vector<std::string>* readLines(std::string filepath) {

  std::vector<std::string> *lines = new std::vector<std::string>();
  std::ifstream file(filepath);
  std::string line;
  if (file.is_open()) {
    while (getline(file, line)) {
      lines->push_back(line);
    }
    file.close();
  }
  return lines;
}

int main(int argc, char *argv[])
{

  // Get command line args
  if (argc < 3) {
    std::cout << "~~ERROR~~" << std::endl;
    std::cout << 
      "Usage: ./bin FILEPATH_OF_LIST_OF_FILES.txt OUTPUT_DIRECTORY" 
      << std::endl;
    return -1;
  }
  std::string mesh_listing_file = argv[1];
  output_dir = argv[2];
  mesh_filepaths = readLines(mesh_listing_file);

  // Bring up viewer and prep for capture
  Eigen::MatrixXd V;
  Eigen::MatrixXi F;
  std::cout << "Press space bar to begin capturing" << std::endl;
  igl::viewer::Viewer viewer;
  viewer.callback_key_down = &key_down;
  viewer.data.set_mesh(V, F);
  viewer.data.set_face_based(false);
  viewer.data.set_normals(viewer.data.V_normals);
  viewer.launch();
}
