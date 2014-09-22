//Dernière mise à jour le 09/01/2011

#ifndef __OBJ_LIB__
#define __OBJ_LIB__

#include <iostream>
#include <fstream>
#include <string>
#include <cmath>
#include <vector>


#include <windows.h>
#include <GL/gl.h>
#include <GL/glu.h>

#include <gmtl/AABox.h>
#include <gmtl/Point.h>
#include <gmtl/Matrix.h>
#include <gmtl/Containment.h>
#include <gmtl/Generate.h>
#include <gmtl/Output.h>
#include <gmtl/Xforms.h>
#include <gmtl/Ray.h>

#include <btBulletDynamicsCommon.h>

std::string doubleSlash(std::string s);
std::string remplacerSlash(std::string s);
std::vector<std::string> splitSpace(std::string s);
std::string get_directory(std::string s);
float* vector2float(std::vector<float>& tableau);

class FloatVector
{
    /*
    Classe FloatVector : simple vecteur XYZ ou XYZA (dans le cas de couleurs).
    */
    public:
        FloatVector(float px=0,float py=0,float pz=0,float pa=0);
        /* FloatVector(float px=0,float py=0,float pz=0,float pa=0);
           Constructeur, prend en paramètres des flottants correspondant respectivement à x, y, z et a.
        */
        ~FloatVector();
        /* ~FloatVector();
           Destructeur, totalement inutile.
        */
        FloatVector operator=(const FloatVector &fv);
        /* FloatVector operator=(const FloatVector &fv);
           Affecte au vecteur courant le contenu du vecteur passé en argument.
           Retourne le vecteur courant ainsi modifié.
        */
        float x,y,z,a;
};

class Material
{
    /*
    Classe Material : définition d'un matériau, composé d'une couleur et d'un nom spécifique.
    */
    public:
        Material(float r,float g,float b,std::string n);
        /* Material(float r,float g,float b,std::string n);
           Constructeur, les trois premiers arguments représentent la couleur RGB du matériau et n est son nom.
        */
        Material(Material *mat);
        /* Material(Material *mat);
           Constructeur alternatif, affecte au matériau courant le contenu du matériau passé en argument.
        */
        ~Material();
        /* ~Material();
           Destructeur, totalement inutile.
        */

        FloatVector coul;
        std::string name;
};

class MeshObj
{
  /*
  Classe MeshObj : définition d'un modèle statique.
  */
public:
  GLfloat center[3];
  gmtl::AABox<float> aabox;
  MeshObj(std::string,MeshObj *first=NULL);
  /* MeshObj(std::string,MeshObj *first=NULL);
    Constructeur, prend en arguments le nom du modèle à charger et le pointeur de la première frame si le modèle appartient à une animation (sinon laissez-le à NULL).
  */
  ~MeshObj();
  /* ~MeshObj();
    Destructeur, libère toute la mémoire qui lui a été allouée.
  */
  void charger_obj(std::string,MeshObj *first=NULL);
  /* void charger_obj(std::string,MeshObj *first=NULL);
    Charge un fichier OBJ et son MTL, prend en arguments le nom du modèle à charger et le pointeur de la première frame si le modèle appartient à une animation (sinon laissez-le à NULL). Cette fonction est appelée par le constructeur.
    Aucune valeur de retour.
  */
  void charger_mtl(std::string);
  /* void charger_mtl(std::string);
    Charge un fichier MTL, prend en argument le nom du fichier à charger. Cette fonction est appelée par charger_obj.
    Aucune valeur de retour.
  */
  void draw_model(bool nor=true,bool tex=false);
  /* void draw_model(bool nor=true,bool tex=false);
    Dessine le modèle, prend en arguments deux booléens représentant respectivement les normales et la texture. Si nor vaut true alors on prend en compte les normales, et si tex vaut true alors on applique la texture.
    Aucune valeur de retour.
  */
  void setMaterialsAndTex(std::vector<Material*> mats,GLuint tex);
  /* void setMaterialsAndTex(std::vector<Material*> mats,GLuint tex);
    Définit directement les matériaux et la texture du modèle, prend en arguments un vector<Material*> et la texture. Cette fonction est appelée par giveMaterialsAndTex.
    Aucune valeur de retour.
  */
  void giveMaterialsAndTex(MeshObj *target);
  /* void giveMaterialsAndTex(MeshObj *target);
    Modifie les matériaux et la texture de target en les remplaçant par ses propres matériaux et sa texture. Cette fonction est appelée par charger_obj uniquement lorsque first!=NULL.
    Aucune valeur de retour.
  */
  void draw_bbox();
  void draw_waabox();
  void make_bbox();
  void recentre();
  void addTransf(const gmtl::Matrix44f& in)
  {
    transf = in*transf;
    updated = false;
  }
  void addPostTransf(const gmtl::Matrix44f& in);
  const gmtl::Matrix44f& getTrans() const
  {
    return transf;
  }
  const gmtl::Matrix44f& getPostTransf() const
  {
    return postTransf;
  }
  void select(bool s = true) {sel = s;}
  bool isSelected() const {return sel;}
  void print() const {
    std::cout << min_x << ' ' << max_x << ' ' << min_y << ' ' << max_y << ' ' << min_z << ' ' << max_z << std::endl;
  }
  const gmtl::AABox<float>& getWaabox()
  {
    if(!updated)
    {
      comp_waabox();
      updated = true;
    }
    return waabox;
  }
  void setBody(btRigidBody *b)
  {
    bulletBody = b;
  }
  btRigidBody* getBody() const
  {
    return bulletBody;
  }
  void zeroBullet()
  {
    bulletBody = 0;
  }
  void printBodyMatrix() const
  {
    if(!bulletBody) return;
    float mat[16];
    btTransform t;
    bulletBody->getMotionState()->getWorldTransform(t);
    t.getOpenGLMatrix(mat);
    for(int i=0; i<16; ++i) std::cout << mat[i] << ' ';
    std::cout << std::endl;
  }
  void printTransf() const
  {
    for(int i=0; i<16; ++i) std::cout << getTrans().mData[i] << ' ';
    std::cout << std::endl;
  }
  void drawBody() const;
  gmtl::Point3f trCentre() const
  {
    return transf * gmtl::Point3f(center[0], center[1], center[2]);
  }
private:
  void comp_waabox();
  GLuint texture;
  int n_data;
  float *vertice,*normals,*textures,*colours;
  gmtl::AABox<float> waabox; // dans les coordonnées du monde virtuel
  bool updated;

  bool sel;
  gmtl::Matrix44f transf;
  gmtl::Matrix44f postTransf;

  btRigidBody *bulletBody;

  GLfloat min_x, max_x, min_y, max_y, min_z, max_z;

  std::vector<FloatVector> ver;

  std::vector<Material*> materiaux;
};


#endif
