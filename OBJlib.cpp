
#include "objlib.h"

using namespace std;

#define DESSINE_TRIANGLES

string doubleSlash(string s)
{
    //Remplace "//" par "/1/".
    string s1="";
    for(unsigned int i=0;i<s.size();i++)
    {
        if(i<s.size()-1&&s[i]=='/'&&s[i+1]=='/')
        {
            s1+="/1/";
            i++;
        }
        else
            s1+=s[i];
    }
    return s1;
}
string remplacerSlash(string s)
{
    //Remplace les '/' par des espaces.
    string ret="";
    for(unsigned int i=0;i<s.size();i++)
    {
        if(s[i]=='/')
            ret+=' ';
        else
            ret+=s[i];
    }
    return ret;
}
vector<string> splitSpace(string s)
{
    //Eclate une chaîne au niveau de ses espaces.
    vector<string> ret;
    string s1="";
    for(unsigned int i=0;i<s.size();i++)
    {
        if(s[i]==' '||i==s.size()-1)
        {
            if(i==s.size()-1)
                s1+=s[i];
            if(s1!="") ret.push_back(s1);
            s1="";
        }
        else
            s1+=s[i];
    }
    return ret;
}
string get_directory(string s)
{
    string s1="",s2="";
    for(unsigned int i=0;i<s.size();i++)
    {
        if(s[i]=='/'||s[i]=='\\')
        {
            s1+=s2+"/";
            s2="";
        }
        else
            s2+=s[i];
    }
    return s1;
}
float* vector2float(vector<float>& tableau)
{
    float* t=NULL;
    t=(float*)malloc(tableau.size()*sizeof(float));
    if(t==NULL||tableau.empty())
    {
        float *t1=(float*)malloc(sizeof(float)*3);
        for(int i=0;i<3;i++)
            t1[i]=0.;
        return t1;
    }

    for(unsigned int i=0;i<tableau.size();i++)
        t[i]=tableau[i];
    return t;
}

FloatVector::FloatVector(float px,float py,float pz,float pa):x(px),y(py),z(pz),a(pa)
{
}
FloatVector::~FloatVector()
{
}
FloatVector FloatVector::operator=(const FloatVector &fv)
{
    x=fv.x;
    y=fv.y;
    z=fv.z;
    a=fv.a;

    return *this;
}

Material::Material(float r,float g,float b,string n):name(n)
{
    coul.x=r;
    coul.y=g;
    coul.z=b;
}
Material::Material(Material *mat)
{
    coul=mat->coul;
    name=mat->name;
}
Material::~Material()
{
}

MeshObj::MeshObj(string s,MeshObj *first)
: updated(true), sel(false), transf(gmtl::MAT_IDENTITY44F), postTransf(gmtl::MAT_IDENTITY44F), initTransf(gmtl::MAT_IDENTITY44F), scale(gmtl::Vec3f(1.f,1.f,1.f)), bulletBody(0)
{
    charger_obj(s,first);
    bodyInitTr.setIdentity();
}
MeshObj::~MeshObj()
{
    free(vertice);
    free(normals);
    free(textures);
    free(colours);

    for(unsigned int i=0;i<materiaux.size();i++)
        delete materiaux[i];
    materiaux.clear();
}
void MeshObj::charger_obj(string nom,MeshObj *first)
{
    vector<FloatVector> nor,tex,col;
    vector<unsigned int> iv,it,in;

    ifstream fichier(nom.c_str(),ios::in);

    string ligne,curname="";

    if(first) //Si ce n'est pas la première frame, on demande à celle-ci de nous passer ses matériaux et sa texture (et elle accepte ^^ )
        first->giveMaterialsAndTex(this);

    if(fichier)
    {
        while(getline(fichier,ligne))
        {
            if(ligne[0]=='v') //Coordonnées de points (vertex, texture et normale)
            {
                if(ligne[1]==' ') //Vertex
                {
                    char x[255],y[255],z[255];
                    sscanf(ligne.c_str(),"v %s %s %s",x,y,z);
                    ver.push_back(FloatVector(strtod(x,NULL),strtod(y,NULL),strtod(z,NULL)));
                }
                else if(ligne[1]=='t') //Texture
                {
                    char x[255],y[255];
                    sscanf(ligne.c_str(),"vt %s %s",x,y);
                    tex.push_back(FloatVector(strtod(x,NULL),strtod(y,NULL)));
                }
                else if(ligne[1]=='n') //Normale
                {
                    char x[255],y[255],z[255];
                    sscanf(ligne.c_str(),"vn %s %s %s",x,y,z);
                    nor.push_back(FloatVector(strtod(x,NULL),strtod(y,NULL),strtod(z,NULL)));
                }
            }
            else if(ligne[0]=='f') //Indice faces
            {
                ligne=doubleSlash(ligne); //On remplace "//" par "/1/" dans toute la ligne
                ligne=remplacerSlash(ligne); //On remplace les '/' par des espaces, ex : pour "f 1/2/3 4/5/6 7/8/9" on obtiendra "f 1 2 3 4 5 6 7 8 9"

                vector<string> termes=splitSpace(ligne.substr(2)); //On éclate la chaîne en ses espaces (le substr permet d'enlever "f ")

                int ndonnees=(int)termes.size()/3;
#ifdef DESSINE_TRIANGLES
                for(int i=0;i<3;i++)
#else
                for(int i=0;i<(ndonnees==3?3:4);i++) //On aurait très bien pu mettre i<ndonnees mais je veux vraiment limiter à 3 ou 4
#endif
                {
                    iv.push_back(strtol(termes[i*3].c_str(),NULL,10)-1);
                    it.push_back(strtol(termes[i*3+1].c_str(),NULL,10)-1);
                    in.push_back(strtol(termes[i*3+2].c_str(),NULL,10)-1);
                }
#ifdef DESSINE_TRIANGLES
                if(ndonnees==4) //ajouter un 2e triangle s'il y a quatre sommets
#else
                if(ndonnees==3) //S'il n'y a que 3 sommets on duplique le dernier pour faire un quad ayant l'apparence d'un triangle
#endif
                {
                    iv.push_back(strtol(termes[0].c_str(),NULL,10)-1);
                    it.push_back(strtol(termes[1].c_str(),NULL,10)-1);
                    in.push_back(strtol(termes[2].c_str(),NULL,10)-1);
#ifdef DESSINE_TRIANGLES
                    iv.push_back(strtol(termes[6].c_str(),NULL,10)-1);
                    it.push_back(strtol(termes[7].c_str(),NULL,10)-1);
                    in.push_back(strtol(termes[8].c_str(),NULL,10)-1);
                    iv.push_back(strtol(termes[9].c_str(),NULL,10)-1);
                    it.push_back(strtol(termes[10].c_str(),NULL,10)-1);
                    in.push_back(strtol(termes[11].c_str(),NULL,10)-1);
#endif
                }

                for(unsigned int i=0;i<materiaux.size();i++)
                    if(materiaux[i]->name==curname)
                    {
#ifdef DESSINE_TRIANGLES
                        for(int j=0;j<(ndonnees==3?4:8);++j)
#else
                        for(int j=0;j<4;j++)
#endif
                            col.push_back(materiaux[i]->coul); //On ajoute la couleur correspondante
                        break;
                    }
            }
            else if(ligne[0]=='m'&&first==NULL)//fichier MTL et si c'est la première frame (comme ça on ne charge pas plusieurs fois le même MTL et la même texture)
                charger_mtl(get_directory(nom)+ligne.substr(7));
            else if(ligne[0]=='u')//utiliser un MTL
                curname=ligne.substr(7);
        }
        fichier.close();

        vector<float> tv(0),tc(0),tn(0),tt(0);
        for(unsigned int i=0;i<iv.size();i++)
            if(iv[i]<ver.size())
            {
                tv.push_back(ver[iv[i]].x);
                tv.push_back(ver[iv[i]].y);
                tv.push_back(ver[iv[i]].z);

                tc.push_back(col[i].x);
                tc.push_back(col[i].y);
                tc.push_back(col[i].z);
                tc.push_back(col[i].a);
            }

        for(unsigned int i=0;i<in.size();i++)
            if(in[i]<nor.size())
            {
                tn.push_back(nor[in[i]].x);
                tn.push_back(nor[in[i]].y);
                tn.push_back(nor[in[i]].z);
            }

        for(unsigned int i=0;i<it.size();i++)
            if(it[i]<tex.size())
            {
                tt.push_back(tex[it[i]].x);
                tt.push_back(tex[it[i]].y);
            }

        vertice=vector2float(tv);
        normals=vector2float(tn);
        textures=vector2float(tt);
        colours=vector2float(tc);
        n_data=iv.size();
        std::cout << nom << ' ' << ver.size() << ' ' << iv.size() << ' ' << tv.size() << std::endl;
        //std::cout << nom << ' ';
        //for(int i=0; i<12; ++i) std::cout << vertice[i] << ' ';
        //std::cout << '\n';
    }
    else
    {
        fprintf(stderr, "Le fichier %s n'a pas pu etre charge...",nom.c_str());
        exit(EXIT_FAILURE);
    }


    //ver.clear();
    nor.clear();
    tex.clear();
    col.clear();

    iv.clear();
    it.clear();
    in.clear();
}
void MeshObj::charger_mtl(string nom)
{
    ifstream fichier(nom.c_str(),ios::in);
    string curname="";
    if(fichier)
    {
        string ligne="";
        while(getline(fichier,ligne))
        {
            if(ligne[0]=='n') //nouveau materiau
                curname=ligne.substr(7);
            else if(ligne[0]=='K'&&ligne[1]=='d') //couleur
            {
                vector<string> termes=splitSpace(ligne.substr(3));
                materiaux.push_back(new Material((float)strtod(termes[0].c_str(),NULL),(float)strtod(termes[1].c_str(),NULL),(float)strtod(termes[2].c_str(),NULL),curname));
            }
            else if(ligne[0]=='m'&&ligne[5]=='d')//map_Kd (texture)
            {
                string f=get_directory(nom)+ligne.substr(7);
               // texture=loadTexture(f.c_str());
            }
            else if(ligne[0]=='d') //opacité
            {
                string n=ligne.substr(2);
                materiaux[materiaux.size()-1]->coul.a=strtod(n.c_str(),NULL);
            }
        }
        fichier.close();
    }
    else
    {
        fprintf(stderr,"Erreur lors de la lecture de %s...",nom.c_str());
        exit(EXIT_FAILURE);
    }
}
void MeshObj::draw_model(bool nor,bool tex)
{
    glEnableClientState(GL_VERTEX_ARRAY);
    if(nor)
        glEnableClientState(GL_NORMAL_ARRAY);
    if(tex)
    {
        glEnableClientState(GL_TEXTURE_COORD_ARRAY);
        glBindTexture(GL_TEXTURE_2D,texture);
    }
    glEnableClientState(GL_COLOR_ARRAY);

    glVertexPointer(3,GL_FLOAT,0,vertice);

    if(tex)
        glTexCoordPointer(2,GL_FLOAT,0,textures);
    if(nor)
        glNormalPointer(GL_FLOAT,0,normals);
    glColorPointer(4,GL_FLOAT,0,colours);

#ifdef DESSINE_TRIANGLES
    glDrawArrays(GL_TRIANGLES,0,n_data);
#else
    glDrawArrays(GL_QUADS,0,n_data);
#endif


    glDisableClientState(GL_COLOR_ARRAY);
    glDisableClientState(GL_TEXTURE_COORD_ARRAY);
    glDisableClientState(GL_NORMAL_ARRAY);
    glDisableClientState(GL_VERTEX_ARRAY);
}
void MeshObj::setMaterialsAndTex(std::vector<Material*> mats,GLuint tex)
{
    materiaux.clear();
    for(unsigned int i=0;i<mats.size();i++)
        materiaux.push_back(new Material(mats[i]));
    texture=tex;
}
void MeshObj::giveMaterialsAndTex(MeshObj *target)
{
    target->setMaterialsAndTex(materiaux,texture);
}
void MeshObj::draw_bbox()
{
  // Define the normals for the cube faces.
   static GLdouble normals[6][3] = {
      { -1.0, 0.0, 0.0 }, { 0.0, 1.0, 0.0 }, { 1.0, 0.0, 0.0 },
      { 0.0, -1.0, 0.0 }, { 0.0, 0.0, 1.0 }, { 0.0, 0.0, -1.0 }
   };
   // access values in the v array declared below.
   static GLint faces[6][4] = {
      { 0, 1, 2, 3 }, { 3, 2, 6, 7 }, { 7, 6, 5, 4 },
      { 4, 5, 1, 0 }, { 5, 6, 2, 1 }, { 7, 4, 0, 3 }
   };

   // Define the vertices based on the min and max points on the cube.
   GLdouble v[8][3];
   v[0][0] = v[1][0] = v[2][0] = v[3][0] = min_x;
   v[4][0] = v[5][0] = v[6][0] = v[7][0] = max_x;
   v[0][1] = v[1][1] = v[4][1] = v[5][1] = min_y;
   v[2][1] = v[3][1] = v[6][1] = v[7][1] = max_y;
   v[0][2] = v[3][2] = v[4][2] = v[7][2] = min_z;
   v[1][2] = v[2][2] = v[5][2] = v[6][2] = max_z;

   glColor4f(0.0f,0.0f,1.0f,0.1f);

   // Draw the cube.
  for ( GLint i = 0; i < 6; ++i )
   {
      glBegin(GL_QUADS);
         glNormal3dv(&normals[i][0]);
         glVertex3dv(&v[faces[i][0]][0]);
         glVertex3dv(&v[faces[i][1]][0]);
         glVertex3dv(&v[faces[i][2]][0]);
         glVertex3dv(&v[faces[i][3]][0]);
      glEnd();
   }
}
void MeshObj::make_bbox()
{
 	min_x = max_x = ver[0].x;
  min_y = max_y = ver[0].y;
  min_z = max_z = ver[0].z;

  for (unsigned int i = 1; i < ver.size(); i++) {
    if (ver[i].x < min_x) min_x = ver[i].x;
    if (ver[i].x > max_x) max_x = ver[i].x;
    if (ver[i].y < min_y) min_y = ver[i].y;
    if (ver[i].y > max_y) max_y = ver[i].y;
    if (ver[i].z < min_z) min_z = ver[i].z;
    if (ver[i].z > max_z) max_z = ver[i].z;
  }

  center[0]=(min_x+max_x)/2;
  center[1]=(min_y+max_y)/2;
  center[2]=(min_z+max_z)/2;

  aabox = gmtl::AABox<float>::AABox( gmtl::Point<float,3>::Point(min_x,min_y,min_z), gmtl::Point<float,3>::Point(max_x,max_y,max_z));
}

void MeshObj::recentre()
{
  transf = gmtl::MAT_IDENTITY44F;

  make_bbox();

  transf(0,3) = -center[0];
  transf(1,3) = -center[1];
  transf(2,3) = -center[2];

  updated = false;
  //make_bbox();

  //for(int i=0; i<ver.size(); ++i)
  //{
  //  ver[i].x = ver[i].x - center[0];
  //  ver[i].y = ver[i].y - center[1];
  //  ver[i].z = ver[i].z - center[2];
  //}

  //make_bbox();
}

void MeshObj::addPostTransf(const gmtl::Matrix44f& in)
{
  gmtl::Matrix44f tmp = gmtl::MAT_IDENTITY44F;
  tmp(0,3) = -transf(0,3);
  tmp(1,3) = -transf(1,3);
  tmp(2,3) = -transf(2,3);
  postTransf = tmp;
  tmp = in;
  tmp(0,3) = 0.f;
  tmp(1,3) = 0.f;
  tmp(2,3) = 0.f;
  postTransf = tmp*postTransf;
  tmp = gmtl::MAT_IDENTITY44F;
  tmp(0,3) = transf(0,3);
  tmp(1,3) = transf(1,3);
  tmp(2,3) = transf(2,3);
  postTransf = tmp*postTransf;
  tmp(0,3) = in(0,3)-transf(0,3);
  tmp(1,3) = in(1,3)-transf(1,3);
  tmp(2,3) = in(2,3)-transf(2,3);
  postTransf = tmp*postTransf;

  updated = false;
}

void MeshObj::comp_waabox()
{
  gmtl::Point3f min = aabox.getMin();
  gmtl::Point3f max = aabox.getMax();
  gmtl::Matrix44f T = postTransf*transf;

  gmtl::Vec4f T0(T(0,0), T(1,0), T(2,0), T(3,0));
  gmtl::Vec4f T1(T(0,1), T(1,1), T(2,1), T(3,1));
  gmtl::Vec4f T2(T(0,2), T(1,2), T(2,2), T(3,2));
  gmtl::Vec4f T3(T(0,3), T(1,3), T(2,3), T(3,3));

  gmtl::Vec4f T0min = T0*min[0];
  gmtl::Vec4f T0max = T0*max[0];
  gmtl::Vec4f T1min = T1*min[1];
  gmtl::Vec4f T1max = T1*max[1];
  gmtl::Vec4f T2min = T2*min[2];
  gmtl::Vec4f T2max = T2*max[2];

  gmtl::Point3f wmin(T(0,3), T(1,3), T(2,3));
  gmtl::Point3f wmax(T(0,3), T(1,3), T(2,3));

  for(int i=0; i<3; ++i)
  {
    if(T0min[i] < T0max[i])
    {
      wmin[i] += T0min[i];
      wmax[i] += T0max[i];
    }
    else
    {
      wmin[i] += T0max[i];
      wmax[i] += T0min[i];
    }

    if(T1min[i] < T1max[i])
    {
      wmin[i] += T1min[i];
      wmax[i] += T1max[i];
    }
    else
    {
      wmin[i] += T1max[i];
      wmax[i] += T1min[i];
    }

    if(T2min[i] < T2max[i])
    {
      wmin[i] += T2min[i];
      wmax[i] += T2max[i];
    }
    else
    {
      wmin[i] += T2max[i];
      wmax[i] += T2min[i];
    }
  }

  waabox.setMin(wmin);
  waabox.setMax(wmax);

  //std::cout << "waaboxmin: " << waabox.mMin[0] << ' ' << waabox.mMin[1] << ' ' << waabox.mMin[2] << std::endl;
  //std::cout << "waaboxmax: " << waabox.mMax[0] << ' ' << waabox.mMax[1] << ' ' << waabox.mMax[2] << std::endl;
}
void MeshObj::draw_waabox()
{
  if(!updated)
  {
    comp_waabox();
    updated = true;
  }

  // Define the normals for the cube faces.
   static GLdouble normals[6][3] = {
      { -1.0, 0.0, 0.0 }, { 0.0, 1.0, 0.0 }, { 1.0, 0.0, 0.0 },
      { 0.0, -1.0, 0.0 }, { 0.0, 0.0, 1.0 }, { 0.0, 0.0, -1.0 }
   };
   // access values in the v array declared below.
   static GLint faces[6][4] = {
      { 0, 1, 2, 3 }, { 3, 2, 6, 7 }, { 7, 6, 5, 4 },
      { 4, 5, 1, 0 }, { 5, 6, 2, 1 }, { 7, 4, 0, 3 }
   };

   // Define the vertices based on the min and max points on the cube.
   GLdouble v[8][3];
   v[0][0] = v[1][0] = v[2][0] = v[3][0] = waabox.mMin[0];
   v[4][0] = v[5][0] = v[6][0] = v[7][0] = waabox.mMax[0];
   v[0][1] = v[1][1] = v[4][1] = v[5][1] = waabox.mMin[1];
   v[2][1] = v[3][1] = v[6][1] = v[7][1] = waabox.mMax[1];
   v[0][2] = v[3][2] = v[4][2] = v[7][2] = waabox.mMin[2];
   v[1][2] = v[2][2] = v[5][2] = v[6][2] = waabox.mMax[2];
 
   glEnable(GL_BLEND);
   glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

   glColor4f(0.0f,1.0f,0.0f,0.1f);

   // Draw the cube.
  for ( GLint i = 0; i < 6; ++i )
   {
      glBegin(GL_QUADS);
         glNormal3dv(&normals[i][0]);
         glVertex3dv(&v[faces[i][0]][0]);
         glVertex3dv(&v[faces[i][1]][0]);
         glVertex3dv(&v[faces[i][2]][0]);
         glVertex3dv(&v[faces[i][3]][0]);
      glEnd();
   }
   glDisable(GL_BLEND);
}
void MeshObj::drawBody() const
{
  if(!bulletBody) return;
  glPushMatrix();
    float mat[16];
    btTransform t;
    getBody()->getMotionState()->getWorldTransform(t);
    t.getOpenGLMatrix(mat);
    glMultMatrixf(mat);
    btVector3 aamin, aamax;
    getBody()->getCollisionShape()->getAabb(btTransform::getIdentity(), aamin, aamax);
    glColor3f(255.0f, 0.f, 0.f);
    glBegin(GL_LINE_STRIP);
      glVertex3f(aamin[0], aamin[1], aamin[2]);
      glVertex3f(aamax[0], aamin[1], aamin[2]);
      glVertex3f(aamax[0], aamin[1], aamax[2]);
      glVertex3f(aamin[0], aamin[1], aamax[2]);
      glVertex3f(aamin[0], aamin[1], aamin[2]);
      glVertex3f(aamin[0], aamax[1], aamin[2]);
      glVertex3f(aamax[0], aamax[1], aamin[2]);
      glVertex3f(aamax[0], aamax[1], aamax[2]);
      glVertex3f(aamin[0], aamax[1], aamax[2]);
      glVertex3f(aamin[0], aamax[1], aamin[2]);
    glEnd();
  glPopMatrix();
}