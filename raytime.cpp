////////////////////////////////////////////////////////////////////////////////
// Temporary code.  Remove this from your raytracer.  This displays
// the contents of a scene file in realtime in a GLUT window.
////////////////////////////////////////////////////////////////////////////////

#include <string>
#include <fstream>
#include <vector>

#include "geom.h"
#ifndef REAL
#include "raytrace.h"
#include "raytime.h"

#define CollisionEpsilon .0001

std::vector<Shape*> Raytime::shapes;
std::vector<Shape*> Raytime::lights;

float Raytime::spin, Raytime::tilt;

float Raytime::ry;

Vector3f Raytime::eye;      // Position of eye for viewing scene
Quaternionf Raytime::orient;   // Represents rotation of -Z to view direction

// Stupid C++ needs callbacks to be static functions.
static Raytime* globalRaytime = nullptr;


////////////////////////////////////////////////////////////////////////


void applyMaterial(Material* mat, const unsigned int program)
{
    int loc = glGetUniformLocation(program, "Kd");
    glUniform3fv(loc, 1, &mat->Kd[0]);

    loc = glGetUniformLocation(program, "Ks");
    glUniform3fv(loc, 1, &mat->Ks[0]);

    loc = glGetUniformLocation(program, "alpha");
    glUniform1f(loc, mat->alpha);

    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, mat->texid);
    loc = glGetUniformLocation(program, "tex");
    glUniform1i(loc, 0);

    loc = glGetUniformLocation(program, "emitter");
    glUniform1i(loc, 0);
}

////////////////////////////////////////////////////////////////////////
// Light: encapsulates a light and communiction with a shader.
////////////////////////////////////////////////////////////////////////
void applyLight(Material* mat, const unsigned int program)
{
    Vector3f Z;

    int loc = glGetUniformLocation(program, "Kd");
    glUniform3fv(loc, 1, &mat->Kd[0]);

    loc = glGetUniformLocation(program, "emitter");
    glUniform1i(loc, 1);
}

////////////////////////////////////////////////////////////////////////
// Shape: encapsulates shapeects to be drawn; uses OpenGL's VAOs
////////////////////////////////////////////////////////////////////////
Shape::Shape(Material* b) : material(b)
{
}

void Shape::draw()
{
}

////////////////////////////////////////////////////////////////////////
// Raytime handles all realtime drawing/interaction
////////////////////////////////////////////////////////////////////////

// Constructor for Raytime.  Initializes OpenGL, GLUT,as well as the
// data elements of the class.
Raytime::Raytime()
{
    // Initialize the OpenGL bindings
    glbinding::Binding::initialize(false);

    globalRaytime = this;
    // Initialize GLUT
    int argc = 0;
    char* argv;

    // Initialize various member attributes
    //materials.push_back(new Material());

    spin = 0.0f;
    tilt = 90.0f;
    front = 0.1f;
    back = 1000.0f;
}

// This function enters the event loop.
void Raytime::run()
{
    cDist = eye.norm();
}
// Called when the scene needs to be redrawn.
void Raytime::DrawScene()
{
}

// Called by GLUT when the window size is changed.
void Raytime::ReshapeWindow(int w, int h)
{
}

// Called by GLUT for keyboard actions.
void Raytime::KeyboardDown(unsigned char key, int x, int y)
{
    printf("key: %c\n", key);
    switch (key) {

    case 27:                    // Escape key
    case 'q':
        glutLeaveMainLoop();
        break;
    }
}

void Raytime::KeyboardUp(unsigned char key, int x, int y)
{
    fflush(stdout);
}

// Called by GLut when a mouse button changes state.
void Raytime::MouseButton(int button, int state, int x, int y)
{
    fflush(stdout);
}

void Raytime::MouseMotion(int x, int y)
{
    // Calculate the change in the mouse position
    int dx = x - mouseX;
    int dy = y - mouseY;


    // Record this position
    mouseX = x;
    mouseY = y;

    // Force a redraw
    glutPostRedisplay();
}


sphere::sphere(const Vector3f center_, const float r, Material* mat) : Shape(mat)
{
    center = center_;
    radius = Vector3f(r, r, r);

    BBox = Bbox(center - radius, center + radius);
    Raytime::shapes.push_back(this);
    if (mat->isLight())
        Raytime::lights.push_back(this);
}

bool sphere::intersect(Ray ray, Intersection& intersect)
{
    const Vector3f Q = ray.Q - center;

    const Vector3f D = ray.D;

    const float b = Q.dot(D);

    float disc = (b * b - Q.dot(Q) + radius.x() * radius.x());

    if (disc < 0) return false;

    disc = sqrt(disc);

    float t0 = -b + disc;
    float t1 = -b - disc;

    if (t0 < CollisionEpsilon && t1 < CollisionEpsilon) return false;

    if (t0 < CollisionEpsilon)
        intersect.t = t1;
    else if (t1 < CollisionEpsilon)
        intersect.t = t0;
    else
        intersect.t = std::min(t0, t1);

    intersect.P = ray.Eval(intersect.t);
    intersect.N = (intersect.P - Center()).normalized();
    intersect.shape = this;
    return true;
}

box::box(const Vector3f base, const Vector3f diag, Material* mat) : Shape(mat), 
s1(-base[0], -base[0] - diag[0], Vector3f(1, 0, 0)), s2(-base[1], -base[1] - diag[1], Vector3f(0, 1, 0)), s3(-base[2], -base[2] - diag[2], Vector3f(0, 0, 1))
{
    BBox = Bbox(base, base + diag);

    Raytime::shapes.push_back(this);
    if (mat->isLight())
        Raytime::lights.push_back(this);
}

bool box::intersect(Ray ray, Intersection& intersect)
{
    Interval i = Interval();
    Interval si1 = s1.intersect(ray);
    Interval si2 = s2.intersect(ray);
    Interval si3 = s3.intersect(ray);

    Interval intervals[3];
    intervals[0] = si1;
    intervals[1] = si2;
    intervals[2] = si3;

    Vector3f directions[3];
    directions[0] = Vector3f(1, 0, 0);
    directions[1] = Vector3f(0, 1, 0);
    directions[2] = Vector3f(0, 0, 1);

    i.t0 = std::max(i.t0, si1.t0);
    i.t1 = std::min(i.t1, si1.t1);

    i.t0 = std::max(i.t0, si2.t0);
    i.t1 = std::min(i.t1, si2.t1);

    i.t0 = std::max(i.t0, si3.t0);
    i.t1 = std::min(i.t1, si3.t1);

    if (i.t0 > i.t1)
        return false;

    if (i.t0 < CollisionEpsilon &&  i.t1 < CollisionEpsilon)
        return false;

    intersect.shape = this;
    if (i.t0 < CollisionEpsilon) {
        intersect.t = i.t1;
    }
    else if (i.t1 < CollisionEpsilon) {
        intersect.t = i.t0;
    }
    else {
        // both positive...
        intersect.t = (std::min)(i.t0, i.t1);
    }

    for (int i = 0; i < 3; ++i)
    {
        if (intervals[i].t0 == intersect.t)
            intersect.N = directions[i];
        if (intervals[i].t1 == intersect.t)
            intersect.N = -directions[i];
    }

    intersect.P = ray.Eval(intersect.t);
    return true;
}


cylinder::cylinder(const Vector3f base_, const Vector3f axis_, const float radius_, Material* mat) : 
Shape(mat), base(base_), axis(axis_), radius(radius_), s1(0,-(axis.norm()), Vector3f(0,0,1))
{
    Vector3f rrr(radius_, radius_, radius_);
    Vector3f L = base_ + rrr;
    Vector3f R = base_ - rrr;
    Vector3f U = axis_ + L;
    Vector3f D = axis_ + R;

    float maxX = std::max(std::max(L.x(), R.x()), std::max(U.x(), D.x()));
    float maxY = std::max(std::max(L.y(), R.y()), std::max(U.y(), D.y()));
    float maxZ = std::max(std::max(L.z(), R.z()), std::max(U.z(), D.z()));

    float minX = std::min(std::min(L.x(), R.x()), std::min(U.x(), D.x()));
    float minY = std::min(std::min(L.y(), R.y()), std::min(U.y(), D.y()));
    float minZ = std::min(std::min(L.z(), R.z()), std::min(U.z(), D.z()));

    BBox = Bbox(Vector3f(minX, minY, minZ), Vector3f(maxX, maxY, maxZ));

    q = Quaternionf::FromTwoVectors(axis, Vector3f::UnitZ());
    Raytime::shapes.push_back(this);
    if (mat->isLight())
        Raytime::lights.push_back(this);
}
bool cylinder::intersect(Ray ray, Intersection& intersect)
{
    Ray og = ray;
    ray.Q = q._transformVector(ray.Q - base);
    ray.D = q._transformVector(ray.D);

    Interval r = Interval();
    Interval i = s1.intersect(ray);

    float dx = ray.D[0];
    float dy = ray.D[1];
    float qx = ray.Q[0];
    float qy = ray.Q[1];

    float a = dx * dx + dy * dy;
    float b = 2 * (dx * qx + dy * qy);
    float c = qx * qx + qy * qy - radius * radius;

    float det = (b * b - (4 * a * c));

    if (det < 0)
        return false;

    float negT = (-b - sqrt(det)) / (2 * a);
    float posT = (-b + sqrt(det)) / (2 * a);

    float t0 = std::max(r.t0, std::max(i.t0, negT));

    float t1 = std::min(r.t1, std::min(i.t1, posT));

    if (t0 > t1 || (t0 <= CollisionEpsilon && t1 <= CollisionEpsilon))
        return false;

    if ((t0 < CollisionEpsilon && t1 > CollisionEpsilon) || (t0 > CollisionEpsilon && t1 < CollisionEpsilon))
    {
        intersect.t = std::max(t0, t1);
    }

    else
    {
        intersect.t = std::min(t0, t1);
    }

    intersect.P = og.Eval(intersect.t);
    intersect.shape = this;

    // On end plate, +- Z
    if (intersect.t == i.t0) {
        intersect.N = Vector3f::UnitZ();
    }
    else if (intersect.t == i.t1) {
        intersect.N = -Vector3f::UnitZ();
    }
    else
    {
        intersect.N = Vector3f(qx + intersect.t * dx, qy + intersect.t * dy, 0.0f);
    }
    intersect.N = (q.conjugate()._transformVector(intersect.N)).normalized();
    return true;
    
}
#endif

Interval Slab::intersect(Ray ray)
{
    if (N.dot(ray.D) != 0)
    {
        float t0 = -(d0 + N.dot(ray.Q)) / N.dot(ray.D);
        float t1 = -(d1 + N.dot(ray.Q)) / N.dot(ray.D);
        return Interval(t0, t1, N, N);
    }
    else
    {
        float s0 = N.dot(ray.Q) + d0;
        float s1 = N.dot(ray.Q) + d1;

        if ((s0 > 0 && s1 < 0) || (s0 < 0 && s1 > 0))
            return Interval();

        else
        {
            Interval i;
            i.empty();
            return i;
        }
    }
}

void Raytime::trianglemesh(MeshData* meshdata)
{
    int size = meshdata->triangles.size();
    for (int i = 0; i < size; ++i)
    {
        Vector3f v0 = meshdata->vertices[meshdata->triangles[i].data()[0]].pnt;
        Vector3f v1 = meshdata->vertices[meshdata->triangles[i].data()[1]].pnt;
        Vector3f v2 = meshdata->vertices[meshdata->triangles[i].data()[2]].pnt;
        Vector3f n0 = meshdata->vertices[meshdata->triangles[i].data()[0]].nrm;
        Vector3f n1 = meshdata->vertices[meshdata->triangles[i].data()[1]].nrm;
        Vector3f n2 = meshdata->vertices[meshdata->triangles[i].data()[2]].nrm;

        n0.normalize();
        n1.normalize();
        n2.normalize();

        triangle* tri = new triangle(v0, v1, v2, n0, n1, n2, meshdata->mat);

        Raytime::shapes.push_back(tri);
        if (meshdata->mat->isLight())
            Raytime::lights.push_back(tri);
    }
}

triangle::triangle(Vector3f v0_, Vector3f v1_, Vector3f v2_, Vector3f n0_, Vector3f n1_, Vector3f n2_, Material* mat) : Shape(mat)
{
    v0 = v0_;
    v1 = v1_;
    v2 = v2_;
    n0 = n0_;
    n1 = n1_;
    n2 = n2_;

    float minX = std::min(v0.x(), std::min(v1.x(), v2.x()));
    float minY = std::min(v0.y(), std::min(v1.y(), v2.y()));
    float minZ = std::min(v0.z(), std::min(v1.z(), v2.z()));

    float maxX = std::max(v0.x(), std::max(v1.x(), v2.x()));
    float maxY = std::max(v0.y(), std::max(v1.y(), v2.y()));
    float maxZ = std::max(v0.z(), std::max(v1.z(), v2.z()));

    BBox = Bbox(Vector3f(minX, minY, minZ), Vector3f(maxX, maxY, maxZ));
}

bool triangle::intersect(Ray ray, Intersection& intersect)
{
    Vector3f E1 = v1 - v0;
    Vector3f E2 = v2 - v0;

    Vector3f p = ray.D.cross(E2);

    float d = p.dot(E1);

    if (d == 0)
        return false;

    Vector3f S = ray.Q - v0;

    float u = (p.dot(S)) / d;

    if (u < 0 || u > 1)
        return false;

    Vector3f q = S.cross(E1);

    float v = (ray.D.dot(q)) / d;

    if (v < 0 || (v + u) > 1)
        return false;

    float t = (E2.dot(q)) / d;

    if (t < CollisionEpsilon)
        return false;

    intersect.t = t;
    intersect.P = ray.Eval(intersect.t);
    intersect.N = (1 - u - v) * n0 + u * n1 + v * n2;
    intersect.N.normalize();
    intersect.shape = this;
    return true;
}

// Called by BVMinimize to intersect the ray with a Shape. This
// should return the intersection t, but should also track
// the minimum t and it's corresponding intersection info.
// Return INF to indicate no intersection.
float Minimizer::minimumOnObject(Shape* obj)
{
    if(obj->intersect(ray, i))
    {
        float val = currIntersect.t;
        if (i.t < currIntersect.t)
        {
            currIntersect = i;
        }
        return val;
    }
    return INF;
}
// Called by BVMinimize to intersect the ray with a Bbox and
// returns the t value. This should be similar to the already
// written box (3 slab) intersection. (The difference being: a
    // distance of zero should be returned if the ray starts within the Bbox.)
// Return INF to indicate no intersection.
float Minimizer::minimumOnVolume(const Bbox& box)
{
    Vector3f L = box.min(); // Box corner
    Vector3f U = box.max(); // Box corner
    Vector3f diag = U - L;

    Slab xSlab(-1 * L.x(), -1 * L.x() - diag.x(), Vector3f(1.0, 0.0, 0.0));
    Slab ySlab(-1 * L.y(), -1 * L.y() - diag.y(), Vector3f(0.0, 1.0, 0.0));
    Slab zSlab(-1 * L.z(), (-1 * L.z() - diag.z()), Vector3f(0.0, 0.0, 1.0));

    Interval intervals[3];
    intervals[0].intersect(ray, xSlab);
    intervals[1].intersect(ray, ySlab);
    intervals[2].intersect(ray, zSlab);

    const float t0 = std::max(0.f, std::max(std::max(intervals[0].t0, intervals[1].t0), intervals[2].t0));
    const float t1 = std::min(INFINITY, std::min(std::min(intervals[0].t1, intervals[1].t1), intervals[2].t1));

    if (t0 > t1) return INF;

    // find which was the min and use that!
    if (t0 < CollisionEpsilon && t1 < CollisionEpsilon) return INF;

    if (t0 < CollisionEpsilon) {
        return 0;
    }
    else if (t1 < CollisionEpsilon) {
        return 0;
    }
    else {
        return std::min(t0, t1);
    }
}

Bbox bounding_box(const Shape* obj)
{
    return obj->bbox(); // Assuming each Shape object has its own bbox method. →
}
