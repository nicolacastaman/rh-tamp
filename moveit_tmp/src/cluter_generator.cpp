#include <fstream>
#include <iostream>

#include <math.h>
#include <stdlib.h> /* srand, rand */
#include <time.h>   /* time */
#include <vector>

struct Circle
{
  double x;
  double y;
};

double dist(double x1, double y1, double x2, double y2)
{
  return sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2));
}

int main(int argc, char** argv)
{

  int num_objects = std::stoi(argv[1]);
  double size_x = 0.50, size_y = 0.60;
  double offset_x = 0.40, offset_y = -0.30;
  // double space = 0.06;
  int num_problems = 15;

  srand(time(0));

  for (int i = 0; i < num_problems; i++)
  {
    std::string path =
        "/home/nicola/Desktop/PDDL/problem_" + std::to_string(i) + ".yaml";

    std::ofstream problem_file;
    problem_file.open(path.c_str());

    problem_file << "num_objects: " << num_objects << "\n";

    std::vector<Circle> circles;

    // create central circke
    Circle circle;
    circle.x = (size_x / 2.0);
    circle.y = (size_y / 2.0);

    problem_file << "object_0" << ": ["
                 << circle.x + offset_x << "," << circle.y + offset_y
                 << "]\n";
    circles.push_back(circle);

    while (circles.size() < num_objects)
    {

      Circle circle;
      circle.x = ((double)rand() / RAND_MAX) * size_x;
      circle.y = ((double)rand() / RAND_MAX) * size_y;

      bool overlapping = false;

      bool near = false;

      // check that it is not overlapping with any existing circle
      // another brute force approach
      for (int j = 0; j < circles.size(); j++)
      {
        Circle existing = circles[j];
        double d = dist(circle.x, circle.y, existing.x, existing.y);

        if (!near && d < 0.12)
          near = true;

        if (d < 0.02 + 0.03) //0.07
        {
          // They are overlapping
          overlapping = true;
          // do not add to array
          break;
        }
      }

      if (circles.empty())
        near = true;

      // add valid circles to array
      if (!overlapping) // && near)
      {

        problem_file << "object_" << circles.size() << ": ["
                     << circle.x + offset_x << "," << circle.y + offset_y
                     << "]\n";

        circles.push_back(circle);
      }
    }

    problem_file.close();
  }

  return 0;
}
