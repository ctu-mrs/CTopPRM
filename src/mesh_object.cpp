/*
 * MeshObject.cpp
 *
 *  Created on: 5. 10. 2014
 *      Author: Robert Pěnička
 */

#include "mesh_object.hpp"

MeshObject::MeshObject(std::string objectname) : position(0, 0, 0, 0, 0, 0) {
  this->name = objectname;
  numFaces = 0;
  numFacesAlloced = 0;
  faces = NULL;
#ifdef COLOR_ARRAY
  colors = NULL;
#endif
  rapidModel = NULL;
  numVertices = 0;
  numVerticesAlloced = 0;
  vertices = NULL;

  numNormals = 0;
  numNormalsAlloced = 0;
  normals = NULL;
  objectStarted = false;
  objectEnded = false;
  // color = Color(0.5,0.5,0.5);
}

MeshObject::~MeshObject() {
  if (vertices != NULL) {
    delete[] vertices;
  }
  if (normals != NULL) {
    delete[] normals;
  }
  if (faces != NULL) {
    delete[] faces;
  }
  if (rapidModel != NULL) {
    delete rapidModel;
  }
  ERROR("MeshObject delete");
}

void MeshObject::BeginObject() {
  objectStarted = true;
#ifdef POSITION_IN_MEAN
  // set position translation to 0 0 0 and use as counter
  this->position.setTranslation(0, 0, 0);
#endif
}

void MeshObject::EndObject() {
  objectEnded = true;
#ifdef POSITION_IN_MEAN
  position.setX(position.getX() / (double(numVertices)));
  position.setY(position.getY() / (double(numVertices)));
  position.setZ(position.getZ() / (double(numVertices)));
#endif

#ifdef OPTIMIZE_MEMORY_SIZE
  // allocate only needed space for vertices
  // vertices
  Point3D *newVertices = new Point3D[numVertices];
  if (!newVertices) {
    // can not allocate
    ERROR("OUT OF MEMORY - CAN NOT ADD VERTICES");
  }
  for (int i = 0; i < numVertices; i++) {
    newVertices[i] = vertices[i];
  }
  delete[] vertices;
  vertices = newVertices;
  numVerticesAlloced = numVertices;
  // normals
  Vector3D *newNormals = new Vector3D[numNormals];
  if (!newNormals) {
    // can not allocate
    ERROR("OUT OF MEMORY - CAN NOT ADD VERTICES");
  }
  for (int i = 0; i < numNormals; i++) {
    newNormals[i] = normals[i];
  }
  delete[] normals;
  normals = newNormals;
  numNormalsAlloced = numNormals;
  // faces
  Face *newFaces = new Face[numFaces];
  if (!newFaces) {
    // can not allocate
    ERROR("OUT OF MEMORY - CAN NOT CREATE ADD FACE");
  }
  for (int i = 0; i < numFaces; i++) {
    newFaces[i] = faces[i];
  }
  // free the old vertices and assign new
  delete[] faces;
  faces = newFaces;
#ifdef COLOR_ARRAY
  // colors
  Color *newColors = new Color[numFaces];
  if (!newColors) {
    // can not allocate
    ERROR("OUT OF MEMORY - CAN NOT CREATE ADD FACE COLOR");
  }
  for (int i = 0; i < numFaces; i++) {
    newColors[i] = colors[i];
  }
  // free the old vertices and assign new
  delete[] colors;
  colors = newColors;
#endif
  // set alocated faces to numfaces
  numFacesAlloced = numFaces;
#endif
}

void MeshObject::setPositionToMean() {
  double meanX = 0;
  double meanY = 0;
  double meanZ = 0;
  for (int i = 0; i < numVertices; i++) {
    meanX += vertices[i].x / (double)numVertices;
    meanY += vertices[i].y / (double)numVertices;
    meanZ += vertices[i].z / (double)numVertices;
  }
  /*
   meanX = meanX / (double) numVertices;
   meanY = meanY / (double) numVertices;
   meanZ = meanZ / (double) numVertices;
   */
  this->position.setTranslation(meanX, meanY, meanZ);
  for (int i = 0; i < numVertices; i++) {
    vertices[i].x -= meanX;
    vertices[i].y -= meanY;
    vertices[i].z -= meanZ;
  }
  INFO("object moved to mean " << meanX << " " << meanY << " " << meanZ);
}

void MeshObject::setPositionToCenter() {
  double maxX = -DBL_MAX;
  double maxY = -DBL_MAX;
  double maxZ = -DBL_MAX;
  double minX = DBL_MAX;
  double minY = DBL_MAX;
  double minZ = DBL_MAX;
  for (int i = 0; i < numVertices; i++) {
    if (vertices[i].x > maxX) {
      maxX = vertices[i].x;
    }
    if (vertices[i].x < minX) {
      minX = vertices[i].x;
    }
    if (vertices[i].y > maxY) {
      maxY = vertices[i].y;
    }
    if (vertices[i].y < minY) {
      minY = vertices[i].y;
    }
    if (vertices[i].z > maxZ) {
      maxZ = vertices[i].z;
    }
    if (vertices[i].z < minZ) {
      minZ = vertices[i].z;
    }
  }
  double meanX = maxX - (maxX - minX) / 2;
  double meanY = maxY - (maxY - minY) / 2;
  double meanZ = maxZ - (maxZ - minZ) / 2;
  this->position.setTranslation(meanX, meanY, meanZ);
  for (int i = 0; i < numVertices; i++) {
    vertices[i].x -= meanX;
    vertices[i].y -= meanY;
    vertices[i].z -= meanZ;
  }
  INFO("object moved to mean " << meanX << " " << meanY << " " << meanZ);
}

double MeshObject::getMinX() {
  double minX = DBL_MAX;
  for (int i = 0; i < numVertices; i++) {
    if (vertices[i].x < minX) {
      minX = vertices[i].x;
    }
  }
  return minX;
}

double MeshObject::getMinY() {
  double minY = DBL_MAX;
  for (int i = 0; i < numVertices; i++) {
    if (vertices[i].y < minY) {
      minY = vertices[i].y;
    }
  }
  return minY;
}

double MeshObject::getMinZ() {
  double minZ = DBL_MAX;
  for (int i = 0; i < numVertices; i++) {
    if (vertices[i].z < minZ) {
      minZ = vertices[i].z;
    }
  }
  return minZ;
}

double MeshObject::getMaxX() {
  double maxX = -DBL_MAX;
  for (int i = 0; i < numVertices; i++) {
    if (vertices[i].x > maxX) {
      maxX = vertices[i].x;
    }
  }
  return maxX;
}

double MeshObject::getMaxY() {
  double maxY = -DBL_MAX;
  for (int i = 0; i < numVertices; i++) {
    if (vertices[i].y > maxY) {
      maxY = vertices[i].y;
    }
  }
  return maxY;
}

double MeshObject::getMaxZ() {
  double maxZ = -DBL_MAX;
  for (int i = 0; i < numVertices; i++) {
    if (vertices[i].z > maxZ) {
      maxZ = vertices[i].z;
    }
  }
  return maxZ;
}

void MeshObject::addVertex(double x, double y, double z) {
  // std::cout << "addVertex " << x << " " << y << " " << z << std::endl;
  if (numVertices == numVerticesAlloced) {
    int n = numVerticesAlloced * 2;
    if (n == 0) {
      n = 1;
    }
    Point3D *newVertices = new Point3D[n];
    if (!newVertices) {
      // can not allocate
      ERROR("OUT OF MEMORY - CAN NOT ADD VERTICES");
    }
    for (int i = 0; i < numVertices; i++) {
      newVertices[i] = vertices[i];
    }
    // free the old vertices and assign new

    delete[] vertices;

    vertices = newVertices;
    numVerticesAlloced = n;
  }
#ifdef POSITION_IN_MEAN
  position.setX(position.getX() + x);
  position.setY(position.getY() + y);
  position.setZ(position.getZ() + z);
#endif
  vertices[numVertices].x = x;
  vertices[numVertices].y = y;
  vertices[numVertices].z = z;
  numVertices++;
}

void MeshObject::addVertex(Point3D point) {
  addVertex(point.x, point.y, point.z);
}

void MeshObject::addNormal(double x, double y, double z) {
  // std::cout << "addVertex " << x << " " << y << " " << z << std::endl;
  if (numNormals == numNormalsAlloced) {
    int n = numNormalsAlloced * 2;
    if (n == 0) {
      n = 1;
    }
    Vector3D *newNormals = new Vector3D[n];
    if (!newNormals) {
      // can not allocate
      ERROR("OUT OF MEMORY - CAN NOT ADD NORMALS");
    }
    for (int i = 0; i < numNormals; i++) {
      newNormals[i] = normals[i];
    }
    // free the old vertices and assign new
    delete[] normals;
    normals = newNormals;
    numNormalsAlloced = n;
  }
  normals[numNormals].x = x;
  normals[numNormals].y = y;
  normals[numNormals].z = z;
  numNormals++;
}

void MeshObject::addNormal(Vector3D normal) {
  addNormal(normal.x, normal.y, normal.z);
}

void MeshObject::dropOutOfXY(double xMin, double xMax, double yMin,
                             double yMax) {
  for (int var = 0; var < numVertices; ++var) {
    Point3D actualPoint = vertices[var];
    if (actualPoint.x < xMin || actualPoint.x > xMax || actualPoint.y < yMin ||
        actualPoint.y > yMax) {
      vertices[var] = vertices[numVertices - 1];
      numVertices = numVertices - 1;
      var--;
    }
  }
}

void MeshObject::addFace(int a, int b, int c) {
  // std::cout << "addFace " << a << " " << b << " " << c << std::endl;
  if (numFaces == numFacesAlloced) {
    int n = numFacesAlloced * 2;
    if (n == 0) {
      n = 1;
    }
    // face
    Face *newFaces = new Face[n];
    if (!newFaces) {
      // can not allocate
      ERROR("OUT OF MEMORY - CAN NOT CREATE ADD FACE");
    }
    for (int i = 0; i < numFaces; i++) {
      newFaces[i] = faces[i];
    }
    // free the old vertices and assign new
    delete[] faces;
    faces = newFaces;
#ifdef COLOR_ARRAY
    // color
    Color *newColors = new Color[n];
    if (!newColors) {
      // can not allocate
      ERROR("OUT OF MEMORY - CAN NOT CREATE ADD FACE COLOR");
    }
    for (int i = 0; i < numFaces; i++) {
      newColors[i] = colors[i];
    }
    // free the old vertices and assign new
    delete[] colors;
    colors = newColors;
#endif
    numFacesAlloced = n;
  }
  faces[numFaces].id1 = a;
  faces[numFaces].id2 = b;
  faces[numFaces].id3 = c;
#ifdef COLOR_ARRAY
  colors[numFaces] = color;
#endif
  numFaces++;
}

void MeshObject::addFace(Face face) { addFace(face.id1, face.id2, face.id3); }

int MeshObject::createRAPIDModel() {
  INFO("createRAPIDModel " << this->name);

  this->rapidModel = new RAPID_model();
  // INFO("rapidModel->BeginModel ");
  this->rapidModel->BeginModel();

  /*	for (int var = 0; var < numFaces; ++var) {
   std::cout << var << std::endl;
   std::cout << vertices[faces[var].id1] << std::endl;
   std::cout << vertices[faces[var].id2] << std::endl;
   std::cout << vertices[faces[var].id3] << std::endl;
   }*/

  INFO("numFaces " << numFaces)
  INFO("numVertices " << numVertices)
  for (int var = 0; var < numFaces; ++var) {
    // ERROR("here is some bad face id");
    // INFO("face "<<var<<" with vertices "<<faces[var].id1<<"
    // "<<faces[var].id2<<" "<<faces[var].id3) exit(1);
    double *p1 = (double *)&(vertices[faces[var].id1]);
    double *p2 = (double *)&(vertices[faces[var].id2]);
    double *p3 = (double *)&(vertices[faces[var].id3]);
    /*
     std::cout << var << std::endl;
     std::cout << p1[0] << "," << p1[1] << "," << p1[2] << std::endl;
     std::cout << p2[0] << "," << p2[1] << "," << p2[2] << std::endl;
     std::cout << p3[0] << "," << p3[1] << "," << p3[2] << std::endl;
     */
    this->rapidModel->AddTri(p1, p2, p3, var);
  }
  // INFO("added triangles")

  int ok = this->rapidModel->EndModel();
  // INFO("ended rapid model")
  return ok;
}

Point3D *MeshObject::getVertices() { return vertices; }
Face *MeshObject::getFaces() { return faces; }

#ifdef COLOR_ARRAY
Color *MeshObject::getColors() { return colors; }
#endif

Vector3D *MeshObject::getNormals() { return normals; }
unsigned int MeshObject::getVerticesNum() { return numVertices; }
unsigned int MeshObject::getFacesNum() { return numFaces; }
unsigned int MeshObject::getNormalsNum() { return numNormals; }
Position3D MeshObject::getPosition() { return this->position; }
Position3D *MeshObject::getPositionPntr() { return &(this->position); }
void MeshObject::setPosition(Position3D pos) { this->position = pos; }
RAPID_model *MeshObject::getRapidModel() { return rapidModel; }

void MeshObject::printInfo() {
  std::cout << "### Object " << this->name << " ###" << std::endl;
  std::cout << "vertices " << numVertices << std::endl;
  std::cout << "normals " << numNormals << std::endl;
  std::cout << "faces " << numFaces << std::endl;
}

void MeshObject::printVerteces() {
  Point3D objectMean(this->position.getX(), this->position.getY(),
                     this->position.getZ());
  for (int verticeIndex = 0; verticeIndex < numVertices; ++verticeIndex) {
    Point3D addedMean = vertices[verticeIndex] + objectMean;
    INFO(addedMean);
  }
}

void MeshObject::printRawVerteces() {
  Point3D objectMean(this->position.getX(), this->position.getY(),
                     this->position.getZ());
  for (int facesIndex = 0; facesIndex < numFaces; ++facesIndex) {
    Face acutalFace = faces[facesIndex];
    Point3D addedMean1 = vertices[acutalFace.id1] + objectMean;
    Point3D addedMean2 = vertices[acutalFace.id2] + objectMean;
    Point3D addedMean3 = vertices[acutalFace.id3] + objectMean;
    INFO(addedMean1 << " " << addedMean2 << " " << addedMean3);
  }
}

std::string MeshObject::getName() { return this->name; }

bool MeshObject::collide(MeshObject *object1, Position3D object1Position,
                         MeshObject *object2, Position3D object2Position) {
  // INFO("collide");
  bool collisionB = true;
  RotationMatrix object1RM = object1Position.getRotationMatrix();
  Vector3D object1TV = object1Position.getTranslationVector();
  RotationMatrix object2RM = object2Position.getRotationMatrix();
  Vector3D object2TV = object2Position.getTranslationVector();
  // INFO("RAPID_Collide");
  // INFO("bject1->getRapidModel() "<<object1->getRapidModel());
  // INFO("bject1->getRapidModel() "<<object2->getRapidModel());
  RAPID_Collide(object1RM.M, (double *)&object1TV, object1->getRapidModel(),
                object2RM.M, (double *)&object2TV, object2->getRapidModel(),
                RAPID_FIRST_CONTACT);
  // INFO("after RAPID_Collide");
  collisionB = !(RAPID_num_contacts == 0);
  return collisionB;
}

bool MeshObject::collide(MeshObject *object1, MeshObject *object2) {
  return MeshObject::collide(object1, object1->getPosition(), object2,
                             object2->getPosition());
}

bool MeshObject::collide(std::vector<MeshObject *> *obstacles,
                         MeshObject *robot, Position3D robotPosition) {
  // INFO("collide begin");
  RotationMatrix robotRotation = robotPosition.getRotationMatrix();
  Vector3D robotTranslation = robotPosition.getTranslationVector();

  for (int var = 0; var < obstacles->size(); ++var) {
    // INFO("test colision with obstacle "<< var);
    Position3D obstaclePosition = (*obstacles)[var]->getPosition();
    RotationMatrix obstacleRotation = obstaclePosition.getRotationMatrix();
    Vector3D obstacleTranslation = obstaclePosition.getTranslationVector();
    bool collisionB = true;
    /*
     INFO("RAPID_Collide bef");
     INFO("robotRotation:");
     INFO(robotRotation);
     INFO("robotTranslation");
     INFO(robotTranslation);
     INFO("obstacleRotation");
     INFO(obstacleRotation);
     INFO("obstacleTranslation");
     INFO(obstacleTranslation);
     */
    RAPID_Collide(robotRotation.M, (double *)&robotTranslation,
                  robot->getRapidModel(), obstacleRotation.M,
                  (double *)&obstacleTranslation,
                  (*obstacles)[var]->getRapidModel(), RAPID_FIRST_CONTACT);
    // INFO("RAPID_Collide aft");
    if (!(RAPID_num_contacts == 0)) {
      return true;
    }
    /*
     if ( MeshObject::collide(robot, robotPosition, (*obstacles)[var],
     (*obstacles)[var]->getPosition()) ) {
     return true;
     }
     */
  }
  // INFO("collide end");
  return false;
}

MeshObject *MeshObject::copyObject(MeshObject *object1) {
  MeshObject *newMeshObject = new MeshObject(object1->getName());
  for (int numVert = 0; numVert < object1->getVerticesNum(); ++numVert) {
    newMeshObject->addVertex(object1->getVertices()[numVert]);
  }
  for (int numFaces = 0; numFaces < object1->getFacesNum(); ++numFaces) {
    newMeshObject->addFace(object1->getFaces()[numFaces]);
  }
  for (int numNorm = 0; numNorm < object1->getNormalsNum(); ++numNorm) {
    newMeshObject->addNormal(object1->getNormals()[numNorm]);
  }
  return newMeshObject;
}

void MeshObject::setMaterialName(std::string materialName) {
  this->materialName = materialName;
}

std::string MeshObject::getMaterialName() { return this->materialName; }

void MeshObject::setColor(RGBColor color) {
  this->color = color;
#ifdef COLOR_ARRAY
  for (int var = 0; var < numFaces; ++var) {
    colors[var] = color;
  }
#endif
}

RGBColor MeshObject::getColor() { return this->color; }
