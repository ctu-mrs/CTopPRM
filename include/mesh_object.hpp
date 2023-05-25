/*
 * MeshObject.h
 *
 *  Created on: 5. 10. 2014
 *      Author: Robert Pěnička
 */

#pragma once

#include <cfloat>
#include <iostream>
#include <string>
#include <vector>

#include "RAPID.H"
#include "common.hpp"

//#define COLOR_ARRAY
//#define POSITION_IN_MEAN
#define OPTIMIZE_MEMORY_SIZE

class MeshObject {
 public:
  MeshObject(std::string objectname = "");
  virtual ~MeshObject();
  int createRAPIDModel();
  void setPositionToMean();
  void setPositionToCenter();
  void printInfo();
  void printVerteces();
  void printRawVerteces();
  void addFace(int a, int b, int c);
  void addFace(Face face);
  void addVertex(double x, double y, double z);
  void addVertex(Point3D point);
  void addNormal(double x, double y, double z);
  void addNormal(Vector3D normal);
  void dropOutOfXY(double xMin, double xMax, double yMin, double yMax);
  Position3D getPosition();
  Position3D* getPositionPntr();
  void setPosition(Position3D pos);
  RAPID_model* getRapidModel();
  Vector3D* getNormals();
  Point3D* getVertices();
  Face* getFaces();
#ifdef COLOR_ARRAY
  Color* getColors();
#endif
  void BeginObject();
  void EndObject();
  unsigned int getVerticesNum();
  unsigned int getFacesNum();
  unsigned int getNormalsNum();
  std::string getName();
  void setMaterialName(std::string materialName);
  std::string getMaterialName();
  void setColor(RGBColor color);
  RGBColor getColor();
  static bool collide(MeshObject* object1, Position3D object1Position,
                      MeshObject* object2, Position3D object2Position);
  static bool collide(MeshObject* object1, MeshObject* object2);
  static bool collide(std::vector<MeshObject*>* obstacles, MeshObject* robot,
                      Position3D robotPosition);
  static MeshObject* copyObject(MeshObject* object1);
  // void saveToMatFile(std::string filename, std::string varName);

  double getMinX();
  double getMinY();
  double getMinZ();

  double getMaxX();
  double getMaxY();
  double getMaxZ();

 private:
  Position3D position;
  RAPID_model* rapidModel;
  Point3D* vertices;
  Vector3D* normals;
  Face* faces;
#ifdef COLOR_ARRAY
  Color* colors;
#endif
  RGBColor color;
  int numVertices;
  int numVerticesAlloced;
  int numNormals;
  int numNormalsAlloced;
  int numFacesAlloced;
  int numFaces;
  bool objectStarted;
  bool objectEnded;
  std::string name;
  std::string materialName;
};
