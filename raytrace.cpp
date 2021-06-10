//////////////////////////////////////////////////////////////////////
// Provides the framework for a raytracer.
////////////////////////////////////////////////////////////////////////

#include <vector>

#ifdef _WIN32
    // Includes for Windows
#include <windows.h>
#include <cstdlib>
#include <limits>
#include <crtdbg.h>
#else
    // Includes for Linux
#endif

#include <ctime>

#include "geom.h"
#include "raytrace.h"
#include "realtime.h"
#include "raytime.h"

#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"
#include <string>
// A good quality *thread-safe* Mersenne Twister random number generator.
#include <random>
std::mt19937_64 RNGen;
std::uniform_real_distribution<> myrandom(0.0, 1.0);
// Call myrandom(RNGen) to get a uniformly distributed random number in [0,1].
#define RussianRoulette 0.8
#define epsilon .000001
Scene::Scene()
{
#ifdef REAL
    realtime = new Realtime();
#else
    raytime = new Raytime();
#endif
}

void Scene::Finit()
{
}

void Scene::triangleMesh(MeshData* mesh)
{
#ifdef REAL
    realtime->triangleMesh(mesh);
#else
    Raytime::trianglemesh(mesh);
#endif
}

Quaternionf Orientation(int i,
    const std::vector<std::string>& strings,
    const std::vector<float>& f)
{
    Quaternionf q(1, 0, 0, 0); // Unit quaternion
    while (i < strings.size()) {
        std::string c = strings[i++];
        if (c == "x")
            q *= angleAxis(f[i++] * Radians, Vector3f::UnitX());
        else if (c == "y")
            q *= angleAxis(f[i++] * Radians, Vector3f::UnitY());
        else if (c == "z")
            q *= angleAxis(f[i++] * Radians, Vector3f::UnitZ());
        else if (c == "q") {
            q *= Quaternionf(f[i + 0], f[i + 1], f[i + 2], f[i + 3]);
            i += 4;
        }
        else if (c == "a") {
            q *= angleAxis(f[i + 0] * Radians, Vector3f(f[i + 1], f[i + 2], f[i + 3]).normalized());
            i += 4;
        }
    }
    return q;
}

////////////////////////////////////////////////////////////////////////
// Material: encapsulates surface properties
void Material::setTexture(const std::string path)
{
    int width, height, n;
    stbi_set_flip_vertically_on_load(true);
    unsigned char* image = stbi_load(path.c_str(), &width, &height, &n, 0);

    // Realtime code below:  This sends the texture in *image to the graphics card.
    // The raytracer will not use this code (nor any features of OpenGL nor the graphics card).
    glGenTextures(1, &texid);
    glBindTexture(GL_TEXTURE_2D, texid);
    glTexImage2D(GL_TEXTURE_2D, 0, n, width, height, 0, GL_RGB, GL_UNSIGNED_BYTE, image);

    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAX_LEVEL, 100);
    glGenerateMipmap(GL_TEXTURE_2D);
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, (int)GL_LINEAR);
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, (int)GL_LINEAR_MIPMAP_LINEAR);
    glBindTexture(GL_TEXTURE_2D, 0);

    stbi_image_free(image);
}

void Scene::Command(const std::vector<std::string>& strings,
    const std::vector<float>& f)
{
    if (strings.size() == 0) return;
    std::string c = strings[0];

    if (c == "screen")
    {
        // syntax: screen width height
#ifdef REAL
        realtime->setScreen(int(f[1]), int(f[2]));
#else
        raytime->setScreen(int(f[1]), int(f[2]));
#endif
        width = int(f[1]);
        height = int(f[2]);
    }

    else if (c == "camera")
    {
        // syntax: camera x y z   ry   <orientation spec>
        // Eye position (x,y,z),  view orientation (qw qx qy qz),  frustum height ratio ry
#ifdef REAL
        realtime->setCamera(Vector3f(f[1], f[2], f[3]), Orientation(5, strings, f), f[4]);
#else
        raytime->setCamera(Vector3f(f[1], f[2], f[3]), Orientation(5, strings, f), f[4]);
#endif   
    }

    else if (c == "ambient")
    {
        // syntax: ambient r g b
        // Sets the ambient color.  Note: This parameter is temporary.
        // It will be ignored once your raytracer becomes capable of
        // accurately *calculating* the true ambient light.
#ifdef REAL
        realtime->setAmbient(Vector3f(f[1], f[2], f[3]));
#else
        raytime->setAmbient(Vector3f(f[1], f[2], f[3]));
#endif  
    }

    else if (c == "brdf")
    {
        // syntax: brdf  r g b   r g b  alpha
        // later:  brdf  r g b   r g b  alpha  r g b ior
        // First rgb is Diffuse reflection, second is specular reflection.
        // third is beer's law transmission followed by index of refraction.
        // Creates a Material instance to be picked up by successive shapes
        if(f.size() > 7)
            currentMat = new Material(Vector3f(f[1], f[2], f[3]), Vector3f(f[4], f[5], f[6]), f[7], Vector3f(f[8], f[9], f[10]), f[11]);
        else
            currentMat = new Material(Vector3f(f[1], f[2], f[3]), Vector3f(f[4], f[5], f[6]), f[7]);
    }

    else if (c == "light")
    {
        // syntax: light  r g b   
        // The rgb is the emission of the light
        // Creates a Material instance to be picked up by successive shapes
        currentMat = new Light(Vector3f(f[1], f[2], f[3]));
    }

    else if (c == "sphere")
    {
        // syntax: sphere x y z   r
        // Creates a Shape instance for a sphere defined by a center and radius
#ifdef REAL
        realtime->sphere(Vector3f(f[1], f[2], f[3]), f[4], currentMat);
#else
        new sphere(Vector3f(f[1], f[2], f[3]), f[4], currentMat);
#endif  
    }

    else if (c == "box")
    {
        // syntax: box bx by bz   dx dy dz
        // Creates a Shape instance for a box defined by a corner point and diagonal vector
#ifdef REAL
        realtime->box(Vector3f(f[1], f[2], f[3]), Vector3f(f[4], f[5], f[6]), currentMat);
#else
        new box(Vector3f(f[1], f[2], f[3]), Vector3f(f[4], f[5], f[6]), currentMat);
#endif      
    }

    else if (c == "cylinder")
    {
        // syntax: cylinder bx by bz   ax ay az  r
        // Creates a Shape instance for a cylinder defined by a base point, axis vector, and radius
#ifdef REAL
        realtime->cylinder(Vector3f(f[1], f[2], f[3]), Vector3f(f[4], f[5], f[6]), f[7], currentMat);
#else
        new cylinder(Vector3f(f[1], f[2], f[3]), Vector3f(f[4], f[5], f[6]), f[7], currentMat);
#endif
    }


    else if (c == "mesh") {
        // syntax: mesh   filename   tx ty tz   s   <orientation>
        // Creates many Shape instances (one per triangle) by reading
        // model(s) from filename. All triangles are rotated by a
        // quaternion (qw qx qy qz), uniformly scaled by s, and
        // translated by (tx ty tz) .
        Matrix4f modelTr = translate(Vector3f(f[2], f[3], f[4]))
            * scale(Vector3f(f[5], f[5], f[5]))
            * toMat4(Orientation(6, strings, f));
        ReadAssimpFile(strings[1], modelTr);
    }


    else {
        fprintf(stderr, "\n*********************************************\n");
        fprintf(stderr, "* Unknown command: %s\n", c.c_str());
        fprintf(stderr, "*********************************************\n\n");
    }
}



const float ag = .2f;
const float ap = 10000;
const float ab = 0.f;

float Chi(float d)
{
    return d > 0 ? 1.f : 0.f;
}

float G1(Vector3f v, Vector3f m, Intersection& i)
{
    float VN = v.dot(i.N);
    float VM = v.dot(m);

    if (VN == 0 || VM == 0)
        return 0.f;
    float tanVTheta = sqrt(1.f - (VN * VN)) / VN;
    if (VN > 1.f)
        return 1.f;

    if (tanVTheta <= 0)
        return 1.f;

    float a = sqrt((ap / 2) + 1) / tanVTheta;

    /*if (a > 1.6)
        return 1.f;*/

    float numerator = 3.535 * a + 2.181 * a * a;
    float denomenator = 1.f + 2.276 * a + 2.577 * a * a;
    float ret = Chi(VM / VN) * ((numerator)/(denomenator));
    return ret;
}

float G(Vector3f Omega0, Vector3f m, Vector3f Omegai, Intersection& i)
{
    return G1(Omegai, m, i) * G1(Omega0, m, i);
}

Color F(float LdotH, Intersection& i)
{
    Color Ks = i.shape->material->Ks;
    float LdotH5 = pow((1 - abs(LdotH)), 5);

    Color ret = Ks + (1 - Ks) * LdotH5;

    return ret;
}

float D(Vector3f m, Intersection& i)
{
    const float num = ap + 2;
    const float denom = 2 * PI;

    return Chi(m.dot(i.N)) * ((num) / (denom)) * pow(m.dot(i.N), ap);
}

Vector3f SampleLobe(Vector3f N, float c, float phi)
{
    float s1 = sqrt(1 - c * c);
    Vector3f K(s1 * cos(phi), s1 * sin(phi), c); // Vector centered around Z-axis
    Quaternionf q = Quaternionf::FromTwoVectors(Vector3f::UnitZ(), N); // q rotates Z to N
    return q._transformVector(K); // K rotated to N's frame
}

float sign(float r)
{
    if (r >= 0)
        return 1;
    else
        return -1;
}

Vector3f SampleBrdf(Vector3f omega0, Vector3f N, Intersection& i)
{
    float s = i.shape->material->Kd.norm() + i.shape->material->Ks.norm() + i.shape->material->Kt.norm();
    float pd = abs(i.shape->material->Kd.norm()) / s;
    float pr = abs(i.shape->material->Ks.norm()) / s;
    float pt = abs(i.shape->material->Kt.norm()) / s;
    float eps1 = myrandom(RNGen);
    float eps2 = myrandom(RNGen);

    float cosTheataM = pow(eps1, 1 / (ap + 1));
    Vector3f m = SampleLobe(N, cosTheataM, 2 * PI * eps2);

    // transmission prob
    float ni = 1.0f, no = 1.0f;
    if (omega0.dot(i.N) > 0.0f) {
        no = i.shape->material->IOR;
    }
    else {
        ni = i.shape->material->IOR;
    }
    const float nn = ni / no;

    if (eps1 < pd)
    {
        return SampleLobe(N, sqrt(eps1), 2 * eps2 * PI);
    }
    else if (eps1 < (pd + pr))
    {
        return 2 * (omega0.dot(m)) * m - omega0;
    }
    else
    {
        // test radiance
        const float wom = omega0.dot(m);
        const float r = 1 - (nn * nn) * (1 - (wom * wom));
        if (r < 0)
        {
            return 2 * (omega0.dot(m)) * m - omega0;
        }
        else
        {
            return (nn * omega0.dot(m) - sign(omega0.dot(N)) * sqrt(r)) * m - (nn * omega0);
        }
    }
}


Color EvalScattering(Vector3f wo, Intersection i, Vector3f wi)
{
    float s = i.shape->material->Kd.norm() + i.shape->material->Ks.norm() + i.shape->material->Kt.norm();
    float pd = abs(i.shape->material->Kd.norm()) / s;
    float pr = abs(i.shape->material->Ks.norm()) / s;
    float pt = abs(i.shape->material->Kt.norm()) / s;
    // diffuse term
    Color Ed = Color(i.shape->material->Kd) / PI;

    // specular term
    //Vector3f L = wi;
    //Vector3f V = wo;
    Vector3f m = (wi + wo).normalized();
    const Color f = F(wi.dot(m), i);
    //float f = F(wi, m, true);
    const float g = G(wo, m, wi, i);
    const float d = D(m, i);
    const float LdotN = std::abs(wi.dot(i.N));
    const float VdotN = std::abs(wo.dot(i.N));

    Color Er = (f * g * d) / (4 * LdotN * VdotN);

    if (pt == 0.0f) {
        return LdotN * (Ed + Er);
    }

    // transmission term

    // beers law, or A(t) = 1
    Color At = Color(1.0f);
    if (wo.dot(i.N) < 0.0f) {
        // At = e^tlogKt
        const float t = 0.5f;
        const float atc = std::exp(t * std::log(i.shape->material->Kt.x()));
        const float atl = std::exp(t * std::log(i.shape->material->Kt.y()));
        const float atq = std::exp(t * std::log(i.shape->material->Kt.z()));
        At = Color(atc, atl, atq);
    }

    // transmission prob
    float ni = 1.0f, no = 1.0f;
    if (wo.dot(i.N) > 0.0f) {
        no = i.shape->material->IOR;
    }
    else {
        ni = i.shape->material->IOR;
    }
    const float nn = ni / no;

    Vector3f m2 = -1 * (no * wi + ni * wo).normalized();

    // test radiance
    const float wom = wo.dot(m2);
    const float r = 1 - (nn * nn) * (1 - (wom * wom));

    Color Et;
    if (r < 0.0f) {
        // total internal reflection
        Et = At * Er;
    }
    else {
        const Color f2 = 1.f - F(wi.dot(m2), i);
        const float g2 = G(wo, m2, wi, i);
        const float d2 = D(m2, i);

        Color left = Color(g2 * d2 * f2) / (LdotN * VdotN);

        const float wim = wi.dot(m2);
        const float rightNum = std::abs(wim) * std::abs(wom) * no * no;
        const float rightDen = (no * wim + ni * wom);

        Et = At * left * (rightNum / (rightDen * rightDen));
    }

    return LdotN * (Ed + Er + Et);
}

float PdfBrdf(Vector3f omega0, Vector3f N, Vector3f omegai, Intersection& i)
{
    float s = i.shape->material->Kd.norm() + i.shape->material->Ks.norm() + i.shape->material->Kt.norm();
    float pd = abs(i.shape->material->Kd.norm()) / s;
    float pr = abs(i.shape->material->Ks.norm()) / s;
    float pt = abs(i.shape->material->Kt.norm()) / s;
    const float Pd = std::abs(omegai.dot(N)) / PI;

    // specular reflection prob
    Vector3f m = (omega0 + omegai).normalized();
    const float end = (1.f / (4 * std::abs(omegai.dot(m))));
    const float Pr = D(m, i) * std::abs(m.dot(N)) * end;

    // transmission prob
    float ni = 1.0f, no = 1.0f;
    if (omega0.dot(N) > 0.0f) {
        no = i.shape->material->IOR;
    }
    else {
        ni = i.shape->material->IOR;
    }
    const float nn = ni / no;

    m = -1 * ((no * omegai + ni * omega0).normalized());

    // test radiance
    const float wom = omega0.dot(m);
    const float r = 1 - (nn * nn) * (1 - (wom * wom));

    float Pt;
    if (r < 0.0f) {
        // total internal reflection
        Pt = Pr;
    }
    else {
        const float wim = omegai.dot(m);
        const float num = (no * no) * std::abs(wim);
        const float den = (no * wim + ni * wom);

        const float end2 = num / (den * den);
        Pt = D(m, i) * std::abs(m.dot(N)) * end2;
    }

    return pd * Pd + pr * Pr + pt * Pt;
}

Intersection Scene::RayTrace(Ray ray)
{
    Minimizer minimizer(ray);

    BVMinimize(Tree, minimizer);

    return minimizer.currIntersect;
}

Color EvalRadiance(Intersection i)
{
    return i.shape->material->Kd;
}

Intersection SampleSphere(Vector3f C, float R, Shape* shape)
{
    float eps1 = myrandom(RNGen);
    float eps2 = myrandom(RNGen);
    float z = 2.f * eps1 - 1.f;
    float r = sqrt(1.f - z * z);
    float a = 2.f * PI * eps2;
    Intersection i;
    i.N = Vector3f(r * cos(a), r * sin(a), z);
    i.P = C + R * i.N;
    i.shape = shape;
    i.t = 0.f;

    return i;
}

float PdfLight(Intersection Q)
{
    float r = reinterpret_cast<sphere*>(Q.shape)->radius.x();
    float area = r * r * 4 * PI;
    return 1.f / (area * Raytime::lights.size());
}

float GeometryFactor(Intersection A, Intersection B)
{
    Vector3f D = A.P - B.P;
    float DD = D.dot(D);
    float AD = A.N.dot(D);
    float BD = B.N.dot(D);

    return abs((AD * BD) / (DD * DD));
}

Intersection SampleLight()
{
    Shape* light = Raytime::lights[0];

    return SampleSphere(light->Center(), reinterpret_cast<sphere*>(light)->radius.x(), light);
}

Color Scene::TracePath(Ray ray)
{
    Color C(0.f, 0.f, 0.f);
    Color W(1.f, 1.f, 1.f);

    Intersection P = RayTrace(ray);
    if (!P.shape)
        return C;

    if (P.shape->material->isLight())
        return EvalRadiance(P);

    Vector3f omega0 = -ray.D;

    while (myrandom(RNGen) <= RussianRoulette)
    {
        Intersection L = SampleLight(); // randomly chosen light and a point on that light
        float p1 = PdfLight(L) / GeometryFactor(P, L); // probability of L, converted to angular

        Ray wi = Ray((L.P - P.P).normalized(), P.P); // direction from P toward L
        float q = PdfBrdf(omega0, P.N, wi.D, P) * RussianRoulette;
        float wmis = (p1 * p1) / (p1 * p1 + q * q);
        Intersection I = RayTrace(wi); // shadow ray
        if (p1 > 0.0f && I.shape != nullptr && L.shape->material->isLight())// && ((I.P - L.P).norm() <= 3.0f)) Add this for more shadows
        { // I exists and is the chosen point on the chosen light
            Color f = EvalScattering(omega0, P, wi.D);
            Color temp = W * wmis * f / p1 * EvalRadiance(L);
            if (!std::isnan(temp.x()) && !std::isnan(temp.y()) && !std::isnan(temp.z())
                && !std::isinf(temp.x()) && !std::isinf(temp.y()) && !std::isinf(temp.z()))
            {
                C += temp;
            }              
        }



        Vector3f omegai = SampleBrdf(omega0, P.N, P).normalized();

        Intersection Q = RayTrace(Ray(omegai, P.P));

        if (!Q.shape)
            break;

        Color f = EvalScattering(omega0, P, omegai);
        float p2 = PdfBrdf(omega0, P.N, omegai, P) * RussianRoulette;

        if (p2 < epsilon)
            break;

        W *= (f / p2);

        if (Q.shape->material->isLight())
        {
            float q2 = PdfLight(Q) / GeometryFactor(P, Q);
            float wmis2 = (p2 * p2) / (p2 * p2 + q2 * q2);
            Color temp = W * wmis2 * EvalRadiance(Q);
            if (std::isnan(temp.x()) || std::isnan(temp.y()) || std::isnan(temp.z())
                || std::isinf(temp.x()) || std::isinf(temp.y()) || std::isinf(temp.z()))
            {
                continue;
            }
            C += temp;
            break;
        }

        P = Q;
        omega0 = -omegai;
    }

    return C;
}
#include "rgbe.h"
void WriteHdrImage(const std::string outName, const int width, const int height, Color* image, int pass)
{
    // Turn image from a 2D-bottom-up array of Vector3D to an top-down-array of floats
    float* data = new float[width * height * 3];
    float* dp = data;
    for (int y = height - 1; y >= 0; --y) {
        for (int x = 0; x < width; ++x) {
            Color pixel = image[y * width + x];
            *dp++ = pixel[0] / pass;
            *dp++ = pixel[1] / pass;
            *dp++ = pixel[2] / pass;
        }
    }

    // Write image to file in HDR (a.k.a RADIANCE) format
    rgbe_header_info info;
    char errbuf[100] = { 0 };

    FILE* fp = fopen(outName.c_str(), "wb");
    info.valid = false;
    int r = RGBE_WriteHeader(fp, width, height, &info, errbuf);
    if (r != RGBE_RETURN_SUCCESS)
        printf("error: %s\n", errbuf);

    r = RGBE_WritePixels_RLE(fp, data, width, height, errbuf);
    if (r != RGBE_RETURN_SUCCESS)
        printf("error: %s\n", errbuf);
    fclose(fp);

    delete data;
}

void Scene::TraceImage(Color* image, const int pass)
{
    std::clock_t start;
    double duration;

    start = std::clock();

    /* Your algorithm here */


#ifdef REAL
    realtime->run();                          // Remove this (realtime stuff)
#else
    raytime->run();
#endif

    Tree = KdBVH<float, 3, Shape*>(Raytime::shapes.begin(), Raytime::shapes.end());
    float rx = raytime->ry * width / height;
    Vector3f X = rx * Raytime::ViewQuaternion()._transformVector(Vector3f::UnitX());
    Vector3f Y = Raytime::ry * Raytime::ViewQuaternion()._transformVector(Vector3f::UnitY());
    Vector3f Z = -1 * Raytime::ViewQuaternion()._transformVector(Vector3f::UnitZ());

    for (int i = 0; i <= pass; ++i)
    {
#pragma omp parallel for schedule(dynamic, 1) // Magic: Multi-thread y loop
        for (int y = 0; y < height; y++)
        {
            float dy = (2 * (y + myrandom(RNGen))) / height - 1.f;
            for (int x = 0; x < width; x++)
            {
                float dx = (2 * (x + myrandom(RNGen))) / width - 1.f;

                Intersection intersect;
                Vector3f direction = dx * X + dy * Y + Z;
                direction.normalize();
                Ray ray = Ray(direction, Raytime::eye);

                /*Intersection i = RayTrace(ray);

                image[y * width + x] += Color((i.N.dot(Raytime::lights[0]->center - i.P)) * i.shape->material->Kd / PI);*/
                image[y * width + x] += TracePath(ray);
            }
        }
        if (i == 1 || i == 8 || i == 64 || i == 512 || i == 1024 || i == 4096)
        {
            std::string file("../common/scenes/test_");
            file += std::to_string(i);
            file += ".hdr";
            WriteHdrImage(file, width, height, image, i);
        }
    }

}
