/*
 * BlenderLoader.h
 *
 *  Created on: 5. 10. 2014
 *      Author: Robert Pěnička
 */

#pragma once
#include <fstream>
#include <iostream>
#include <vector>
//#include <glm/glm.hpp>
//#include <GL/glu.h>
//#include "CLog.h"
#include <sstream>
#include <string>

#include "mesh_object.hpp"

#define FILE_HEADER                                                  \
  "# Blender v2.72 (sub 0) OBJ File: 'goalConfigurationCubes.blend'" \
    << std::endl                                                     \
    << "# www.blender.org" << std::endl

class BlenderLoader {
 public:
  BlenderLoader();
  virtual ~BlenderLoader();
  static std::vector<MeshObject*> loadObjectsFromFile(std::string filename);
  static void saveObjectsToFile(const char* filename, std::vector<MeshObject*>,
                                bool addObjectPosition = true);

 private:
  static void loadMaterialForObjectFromFile(
    std::vector<MeshObject*> meshObjects, std::string matLibFilename);
};
