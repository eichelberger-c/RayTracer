#ifndef _H_RAYTIME
#define _H_RAYTIME

#ifndef REAL
////////////////////////////////////////////////////////////////////////////////
// Temporary code.  Remove this from your raytracer.  This displays
// the contents of a scene file in Raytime in a GLUT window.
////////////////////////////////////////////////////////////////////////////////

#include <string>
#include <fstream>
#include <vector>
#include <algorithm>

#include <Eigen/StdVector>
#include <Eigen_unsupported/Eigen/BVH>
const float INF = std::numeric_limits<float>::infinity();
typedef Eigen::AlignedBox<float, 3> Bbox; // The BV type provided by Eigen




#include <glbinding/gl/gl.h>
#include <glbinding/Binding.h>
using namespace gl;
#include <freeglut.h>
const float Radians = PI / 180.0f;    // Convert degrees to radians


////////////////////////////////////////////////////////////////////////
// Shader programming class;  Encapsulates a OpenGL Shader.
////////////////////////////////////////////////////////////////////////



class ShaderProgram
{
public:
    int program;

    void CreateProgram() { program = glCreateProgram(); }
    void Use() { glUseProgram(program); }
    void Unuse() { glUseProgram(0); }
    void CreateShader(const std::string fname, const GLenum type);
    void LinkProgram();

};

class Intersection
{
    public:
        float t = INF;
        Shape* shape = nullptr;
        Vector3f P = Vector3f(0,0,0);
        Vector3f N = Vector3f(0,0,0);
};

class Ray
{
public:
    Ray(Vector3f D_, Vector3f Q_) { Q = Q_; D = D_; }
    Vector3f Eval(float t) { return Q + t * D; }

    Vector3f Q;
    Vector3f D;
};
class Interval;
class Slab
{
public:
    Slab(float d0_, float d1_, Vector3f N_) { d0 = d0_; d1 = d1_; N = N_; }
    float d0;
    float d1;
    Vector3f N;


    Interval intersect(Ray ray);
private:
};



class Interval
{
    public:
        Interval() { t0 = 0; t1 = std::numeric_limits<float>::infinity(); }
        Interval(float t0_, float t1_, Vector3f N0_, Vector3f N1_) 
        {   
            if (t0_ > t1_)
            {
                t1 = t0_;
                N1 = N0_;
                t0 = t1_;
                N0 = N1_;
            }
            else
            {
                t0 = t0_;
                N0 = N0_;
                t1 = t1_;
                N1 = N1_;
            }
        } 
        void empty() { t0 = 1; t1 = 0; }
        float first()
        {
            if ((t0 > 0 && t1 < 0) || (t0 < 0 && t1 > 0))
                return t0 > t1 ? t0 : t1;

            return t0 < t1 ? t0 : t1;
        }
        virtual void intersect(Ray ray, Slab slab)
        {
            *this = slab.intersect(ray);
        }
        float t0;
        float t1;
        Vector3f N0;
        Vector3f N1;
};




////////////////////////////////////////////////////////////////////////
// Shape: encapsulates shapeects to be drawn; uses OpenGL's VAOs
////////////////////////////////////////////////////////////////////////
class Shape
{
public:
    MeshData* meshdata;
    Matrix4f modelTR;
    Material* material;
    Vector3f center;
    unsigned int vao;
    Shape(Material* b);
    void draw();
    Bbox BBox;
    Vector3f Center() { return center; }

    Bbox bbox() const { return BBox; };
    virtual bool intersect(Ray ray, Intersection& intersect) = 0;
};

////////////////////////////////////////////////////////////////////////
// Raytime handles all Raytime drawing/interaction
////////////////////////////////////////////////////////////////////////
class Raytime
{
public:
    // Camera/viewing parameters
    Vector3f ambient;
    static Vector3f eye;      // Position of eye for viewing scene
    static Quaternionf orient;   // Represents rotation of -Z to view direction
    static float ry;
    float front, back;
    static float spin, tilt;
    float cDist;              // Distance from eye to center of scene
    //float lightSpin, lightTilt, lightDist;

    int mouseX, mouseY;
    bool shifted;
    bool leftDown;
    bool middleDown;
    bool rightDown;

    ShaderProgram lighting;

    int width, height;
    void setScreen(const int _width, const int _height) { width = _width;  height = _height; }
    void setCamera(const Vector3f& _eye, const Quaternionf& _o, const float _ry)
    {
        eye = _eye; orient = _o; ry = _ry;
    }
    void setAmbient(const Vector3f& _a) { ambient = _a; }

    static std::vector<Shape*> shapes;
    static std::vector<Shape*> lights;

    static Quaternionf ViewQuaternion() {
        Quaternionf q = angleAxis((tilt - 90.0f) * Radians, Vector3f(1, 0, 0))
            * orient.conjugate()
            * angleAxis(spin * Radians, Vector3f(0, 0, 1));
        return q.conjugate();
    }

    Vector3f ViewDirection() {
        return ViewQuaternion().toRotationMatrix() * Vector3f(0.0f, 0.0f, -1.0f);
    }

    void DrawScene();
    void ReshapeWindow(int w, int h);
    void KeyboardUp(unsigned char key, int x, int y);
    void KeyboardDown(unsigned char key, int x, int y);
    void MouseButton(int button, int state, int x, int y);
    void MouseMotion(int x, int y);

    Raytime();
    void run();

    static void trianglemesh(MeshData* meshdata);
};

class triangle : public Shape
{
    public:
        triangle(Vector3f, Vector3f, Vector3f, Vector3f, Vector3f, Vector3f, Material* mat);
        bool intersect(Ray ray, Intersection& intersect);
    private:
        Vector3f v0;
        Vector3f v1;
        Vector3f v2;

        Vector3f n0;
        Vector3f n1;
        Vector3f n2;
};

class cylinder : public Shape
{
    public:
        cylinder(const Vector3f base, const Vector3f axis, const float radius, Material* mat);
        bool intersect(Ray ray, Intersection& intersect);
    private:
        Vector3f base;
        Vector3f axis;
        float radius;
        Slab s1;

        Eigen::Quaternionf q;
};

class Minimizer
{
public:
    typedef float Scalar; // KdBVH needs Minimizer::Scalar defined
    Ray ray;
    Intersection i;

    Intersection currIntersect;
    // Stuff to track the minimal t and its intersection info
    // Constructor
    Minimizer(const Ray& r) : ray(r) {}
    // Called by BVMinimize to intersect the ray with a Shape. This
    // should return the intersection t, but should also track
    // the minimum t and it's corresponding intersection info.
    // Return INF to indicate no intersection.
    float minimumOnObject(Shape* obj);
    // Called by BVMinimize to intersect the ray with a Bbox and
    // returns the t value. This should be similar to the already
    // written box (3 slab) intersection. (The difference being: a
     // distance of zero should be returned if the ray starts within the Bbox.)
    // Return INF to indicate no intersection.
    float minimumOnVolume(const Bbox& box);
};


class sphere : public Shape
{
public:
    sphere(const Vector3f center, const float r, Material* mat);
    bool intersect(Ray ray, Intersection& intersect);
    Vector3f radius;
private:

};

class box : public Shape
{
    public:
        box(const Vector3f base, const Vector3f diag, Material* mat);
        bool intersect(Ray ray, Intersection& intersect);
    private:
        Slab s1;
        Slab s2;
        Slab s3;
};
Bbox bounding_box(const Shape* obj);

#endif

#endif


