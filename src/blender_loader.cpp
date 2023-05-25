/*
 * BlenderLoader.cpp
 *
 *  Created on: 5. 10. 2014
 *      Author: Robert Pěnička
 */

#include "blender_loader.hpp"

BlenderLoader::BlenderLoader() {
  // TODO Auto-generated constructor stub
}

BlenderLoader::~BlenderLoader() {
  // TODO Auto-generated destructor stub
}

std::vector<MeshObject*> BlenderLoader::loadObjectsFromFile(
  std::string filename) {
  INFO("loadObjectsFromFile " << filename);
  unsigned int actual_vertex_id = 1;
  unsigned int actual_normal_id = 1;
  unsigned int substract_id = actual_vertex_id;
  std::vector<MeshObject*> loadedObjects;
  std::ifstream in(filename, std::ifstream::in);
  std::string fullFileName = std::string(filename);
  std::string filePath = getPath(fullFileName);
  if (!in) {
    ERROR("Cannot open " << filename);
    exit(1);
  } else {
    INFO("Loading " << filename);
    std::string materialLib;
    std::string line;
    // int lineNum = 1;
    while (getline(in, line)) {
      // WINFO(lineNum);
      // lineNum++;
      if (line.substr(0, 2) == "o ") {
        std::istringstream s(line.substr(2));
        std::string objectName;
        s >> objectName;
        // std::cout << "o " << objectName << std::endl;
        MeshObject* newObject = new MeshObject(objectName);
        // std::cout << "newObject " << std::endl;
        newObject->BeginObject();
        loadedObjects.push_back(newObject);
        // std::cout << "push_back " << std::endl;
        substract_id = actual_vertex_id;
      } else if (line.substr(0, 2) == "v ") {
        std::istringstream s(line.substr(2));
        double x, y, z;
        s >> x;
        s >> y;
        s >> z;
        Point3D n(x, y, z);

        // std::cout <<"vertex "<< n << std::endl;
        loadedObjects.back()->addVertex(x, y, z);
        actual_vertex_id++;
      } else if (line.substr(0, 2) == "vn ") {
        std::istringstream s(line.substr(2));
        double x, y, z;
        s >> x;
        s >> y;
        s >> z;
        // Point3D p(x, y, z);
        // std::cout << p << std::endl;
        loadedObjects.back()->addNormal(x, y, z);
        actual_normal_id++;
      } else if (line.substr(0, 2) == "f ") {
        for (size_t i = 0; i < line.size(); ++i) {
          if (line[i] == '/') {
            line.replace(i, 1, " ");
          }
        }
        std::istringstream s(line.substr(2));

        unsigned int a, b, c = 0;
        unsigned int an, bn, cn = 0;
        s >> a;
        s >> an;
        s >> b;
        s >> bn;
        s >> c;
        s >> cn;
        a = a - substract_id;
        b = b - substract_id;
        c = c - substract_id;
        // std::cout << line << std::endl;
        // std::cout << "f " << a << " " << b << " " << c << std::endl;
        // std::cout << substract_id << std::endl;
        // std::cout<< a << " " << b << " " << c <<std::endl;
        //	loadedObjects.back().faces.push_back(a);
        //	loadedObjects.back().faces.push_back(b);
        //	loadedObjects.back().faces.push_back(c);
        loadedObjects.back()->addFace(a, b, c);

      } else if (line.substr(0, 7) == "mtllib ") {
        std::istringstream s(line.substr(7));
        std::string materialFileName;
        s >> materialFileName;
        materialLib = filePath + PATH_SEPARATOR + materialFileName;
        INFO("found material library " << materialLib);
      } else if (line.substr(0, 7) == "usemtl ") {
        std::istringstream s(line.substr(7));
        std::string materialName;
        s >> materialName;
        loadedObjects.back()->setMaterialName(materialName);
        INFO("found material " << materialName << " for object "
                               << loadedObjects.back()->getName());
      } else if (line[0] == '#') { /* ignoring this line */
      } else {                     /* ignoring this line */
      }
    }

    // INFO("exit in blender loader");
    // exit(1);
    INFO_GREEN("file readed ");

    INFO("loadedObjects.size " << loadedObjects.size());
    for (int var = 0; var < loadedObjects.size(); ++var) {
      INFO("object " << var << " " << loadedObjects[var]->getName()
                     << " has faces " << loadedObjects[var]->getFacesNum());
      /*
       for (int i = 0; i < loadedObjects[var]->getFacesNum(); ++i) {
       GLushort ia = loadedObjects[var]->getFaces()[i].id1;
       GLushort ib = loadedObjects[var]->getFaces()[i].id2;
       GLushort ic = loadedObjects[var]->getFaces()[i].id3;
       INFO("glushort "<<i)
       INFO("verties num "<<loadedObjects[var]->getVerticesNum())
       INFO("ia "<<ia <<" ib "<<ib<<" ic "<<ic)
       Vector3D AB = loadedObjects[var]->getVertices()[ib]
       - loadedObjects[var]->getVertices()[ia];
       Vector3D AC = loadedObjects[var]->getVertices()[ic]
       - loadedObjects[var]->getVertices()[ia];
       Vector3D crossProd = crossProduct(AB, AC);
       Vector3D normalized = normalize(crossProd);
       INFO("add normal")
       loadedObjects[var]->addNormal(normalized);
       }
       */
      loadedObjects[var]->EndObject();
    }
    // load materials
    loadMaterialForObjectFromFile(loadedObjects, materialLib);
  }
  INFO("DONE");
  // exit(1);
  return loadedObjects;
}

void BlenderLoader::loadMaterialForObjectFromFile(
  std::vector<MeshObject*> meshObjects, std::string matLibFilename) {
  INFO("loadMaterialForObjectFromFile " << matLibFilename);
  std::ifstream in(matLibFilename.c_str(), std::ifstream::in);
  if (!in) {
    ERROR("Cannot open " << matLibFilename);
  } else {
    INFO("Loading " << matLibFilename);
    std::vector<std::string> loadedMaterialNames;
    std::vector<RGBColor> loadedMaterialColors;
    std::string line;
    std::string actualMaterial;
    while (getline(in, line)) {
      if (line.substr(0, 7) == "newmtl ") {
        std::istringstream s(line.substr(7));
        std::string materialName;
        s >> materialName;
        INFO("found material " << materialName);
        actualMaterial = materialName;
        loadedMaterialNames.push_back(materialName);
      } else if (line.substr(0, 3) == "Kd ") {
        std::istringstream s(line.substr(3));
        double r, g, b;
        s >> r;
        s >> g;
        s >> b;
        RGBColor c(r, g, b);
        loadedMaterialColors.push_back(c);
        INFO("found color " << c << " for material " << actualMaterial);
      } else { /* ignoring this line */
      }
    }

    // load colors to objects
    int objectIndex, materialIndex;
    for (objectIndex = 0; objectIndex < meshObjects.size(); ++objectIndex) {
      for (materialIndex = 0; materialIndex < loadedMaterialNames.size();
           ++materialIndex) {
        if (meshObjects[objectIndex]->getMaterialName() ==
            loadedMaterialNames[materialIndex]) {
          RGBColor color;
          color = loadedMaterialColors[materialIndex];
          meshObjects[objectIndex]->setColor(color);
          INFO("set color " << color << " to object "
                            << meshObjects[objectIndex]->getName());
        }
      }
    }
  }
  INFO("DONE");
}

void BlenderLoader::saveObjectsToFile(const char* filename,
                                      std::vector<MeshObject*> objects,
                                      bool addObjectPosition) {
  INFO("saveObjectsToFile");
  std::ofstream myfile;
  myfile.open(filename);
  if (myfile.fail()) {
    // can not open file
    ERROR("can not open file " << filename);
    return;
  }
  INFO("save file opened " << filename);
  myfile << FILE_HEADER;
  unsigned int actual_vertex_id = 1;
  unsigned int substract_id = actual_vertex_id;
  for (int objectIndex = 0; objectIndex < objects.size(); ++objectIndex) {
    INFO("save objectIndex " << objectIndex);
    substract_id = actual_vertex_id;
    myfile << "o " << objects[objectIndex]->getName() << std::endl;
    INFO("name saved ");
    Point3D objectMean(objects[objectIndex]->getPosition().getX(),
                       objects[objectIndex]->getPosition().getY(),
                       objects[objectIndex]->getPosition().getZ());
    int pritAfter = round(objects[objectIndex]->getVerticesNum() / 10);
    for (int verticeIndex = 0;
         verticeIndex < objects[objectIndex]->getVerticesNum();
         ++verticeIndex) {
      Point3D addedMean;
      if (addObjectPosition) {
        addedMean =
          objects[objectIndex]->getVertices()[verticeIndex] + objectMean;
      } else {
        addedMean = objects[objectIndex]->getVertices()[verticeIndex];
      }
      myfile << "v" << addedMean << std::endl;
      actual_vertex_id++;
      if (actual_vertex_id % pritAfter == 0) {
        INFO("saved vertices " << actual_vertex_id << "/"
                               << objects[objectIndex]->getVerticesNum());
      }
    }
    INFO("saved vertices " << actual_vertex_id << "/"
                           << objects[objectIndex]->getVerticesNum());
    for (int faceIndex = 0; faceIndex < objects[objectIndex]->getFacesNum();
         ++faceIndex) {
      myfile << "f "
             << (objects[objectIndex]->getFaces()[faceIndex].id1 + substract_id)
             << " "
             << (objects[objectIndex]->getFaces()[faceIndex].id2 + substract_id)
             << " "
             << (objects[objectIndex]->getFaces()[faceIndex].id3 + substract_id)
             << std::endl;
    }
  }
  myfile.close();
}
