#include "ofApp.h"

namespace rt {
	class Film {
	public:
		Film(int w, int h) :_w(w), _h(h), _pixels(h*w) {
			for (int i = 0; i < _pixels.size(); ++i) {
				_pixels[i].random = Xor(i + 1);
				for (int j = 0; j < 100; ++j) {
					_pixels[i].random.generate();
				}
			}
		}
		int width() const {
			return _w;
		}
		int height() const {
			return _h;
		}

		void add(int x, int y, Vec3 c) {
			int index = y * _w + x;
			_pixels[index].color += c;
			_pixels[index].sample++;
		}

		struct Pixel {
			int sample = 0;
			Vec3 color;
			Xor random;
		};
		const Pixel *pixel(int x, int y) const {
			return _pixels.data() + y * _w + x;
		}
		Pixel *pixel(int x, int y) {
			return _pixels.data() + y * _w + x;
		}
	private:
		int _w = 0;
		int _h = 0;
		std::vector<Pixel> _pixels;
	};

	class Camera {
	public:
		Camera(int w, int h):_film(w, h) {
			_proj = glm::perspectiveFov(45.0, (double)w, (double)h, 1.0, 100.0);
			auto vp = Vec4(0.0, 0.0, (double)w, h);
			_UR = glm::unProject(Vec3(0.0, (double)h, 0.0), Mat4(), _proj, vp);
			auto r = glm::unProject(Vec3((double)w, (double)h, 0.0), Mat4(), _proj, vp);
			auto b = glm::unProject(Vec3(0.0, 0.0, 0.0), Mat4(), _proj, vp);
			_dirRright = r - _UR;
			_dirBottom = b - _UR;
		}

		Film *film() {
			return &_film;
		}
		const Film *film() const {
			return &_film;
		}

		Ray generateRay(double filmx, double filmy) const {
			Ray ray;
			ray.d = glm::normalize(_UR + _dirRright * (filmx / _film.width()) + _dirBottom * (filmy / _film.height()));
			return ray;
		}
	private:
		Film _film;
		Mat4 _proj;

		Vec3 _UR;
		Vec3 _dirRright;
		Vec3 _dirBottom;
	};

	// 
	struct Triangle {
		Triangle() {}
		Triangle(Vec3 a, Vec3 b, Vec3 c) {
			v[0] = a;
			v[1] = b;
			v[2] = c;
		}
		Vec3 v[3];
	};

	inline Vec3 triangle_normal(const Triangle &triangle, bool isback) {
		Vec3 e1 = triangle.v[1] - triangle.v[0];
		Vec3 e2 = triangle.v[2] - triangle.v[0];
		return glm::normalize(isback ? glm::cross(e2, e1) : glm::cross(e1, e2));
	}

	inline bool intersect_triangle(const Ray &ray, const Triangle &triangle, TriangleIntersection *intersection, double tmin = std::numeric_limits<double>::max()) {
		const Vec3 &v0 = triangle.v[0];
		const Vec3 &v1 = triangle.v[1];
		const Vec3 &v2 = triangle.v[2];

		Vec3 baryPosition;

		Vec3 e1 = v1 - v0;
		Vec3 e2 = v2 - v0;

		Vec3 p = glm::cross(ray.d, e2);

		double a = glm::dot(e1, p);

		double f = 1.0 / a;

		Vec3 s = ray.o - v0;

		Vec3 q = glm::cross(s, e1);
		baryPosition.z = f * glm::dot(e2, q);
		if (baryPosition.z < 0.0 || tmin < baryPosition.z) {
			return false;
		}

		baryPosition.x = f * glm::dot(s, p);
		if (baryPosition.x < 0.0 || baryPosition.x > 1.0) {
			return false;
		}

		baryPosition.y = f * glm::dot(ray.d, q);
		if (baryPosition.y < 0.0 || baryPosition.y + baryPosition.x > 1.0) {
			return false;
		}

		bool isback = a < 0.0;

		intersection->tmin = baryPosition.z;
		intersection->normal = triangle_normal(triangle, isback);
		intersection->uv = Vec2(baryPosition.x, baryPosition.y);
		return true;
	}


	//inline bool intersect_shere(const Ray &ray, const Sphere &s, Intersection *intersection, double tmin = std::numeric_limits<double>::max()) {
	//	Vec3 m = ray.o - s.center;
	//	double b = glm::dot(m, ray.d);
	//	double radius_squared = s.radius * s.radius;
	//	double c = glm::dot(m, m) - radius_squared;
	//	if (c > 0.0 && b > 0.0) {
	//		return false;
	//	}
	//	double discr = b * b - c;
	//	if (discr < 0.0) {
	//		return false;
	//	}

	//	double sqrt_discr = glm::sqrt(discr);
	//	double tmin_near = -b - sqrt_discr;
	//	if (0.0 < tmin_near) {
	//		// back = false
	//		auto p = ray.o + ray.d * tmin_near;
	//		intersection->tmin = tmin_near;
	//		intersection->normal = glm::normalize(p - s.center);
	//		return true;
	//	}
	//	double tmin_far = -b + sqrt_discr;
	//	if (0.0 < tmin_far) {
	//		// back = true
	//		auto p = ray.o + ray.d * tmin_far;
	//		intersection->tmin = tmin_far;
	//		intersection->normal = glm::normalize(s.center - p);
	//		return true;
	//	}
	//	return false;
	//}

	// directionを yaxisに基底変換
	inline Vec3 basisTransform(const Vec3 &direction, const Vec3 &yaxis) {
		Vec3 xaxis;
		Vec3 zaxis;
		if (0.999 < glm::abs(yaxis.z)) {
			xaxis = glm::normalize(glm::cross(Vec3(0.0, -1.0, 0.0), yaxis));
		}
		else {
			xaxis = glm::normalize(glm::cross(Vec3(0.0, 0.0, 1.0), yaxis));
		}
		zaxis = glm::cross(xaxis, yaxis);
		return direction.x * xaxis + direction.y * yaxis + direction.z * zaxis;
	}

	class ConelboxObject {
	public:
		ConelboxObject() {
			float z = -3.0;
			double size = 2.2f;
			double hsize = size * 0.5;
			LambertianMaterial R(Vec3(), Vec3(0.75, 0.25, 0.25));
			LambertianMaterial G(Vec3(), Vec3(0.25, 0.25, 0.75));
			LambertianMaterial W(Vec3(), Vec3(0.75));

			Vec3 ps[] = {
				Vec3(-hsize, hsize, -hsize),
				Vec3(-hsize, hsize, hsize),
				Vec3(hsize, hsize, hsize),
				Vec3(hsize, hsize, -hsize),

				Vec3(-hsize, -hsize, -hsize),
				Vec3(-hsize, -hsize, hsize),
				Vec3(hsize, -hsize, hsize),
				Vec3(hsize, -hsize, -hsize)
			};
			for (int i = 0; i < sizeof(ps) / sizeof(ps[0]); ++i) {
				ps[i].z += z;
			}

			triangles.emplace_back(Triangle(ps[0], ps[1], ps[4]), R);
			triangles.emplace_back(Triangle(ps[1], ps[5], ps[4]), R);

			triangles.emplace_back(Triangle(ps[2], ps[3], ps[7]), G);
			triangles.emplace_back(Triangle(ps[6], ps[2], ps[7]), G);

			triangles.emplace_back(Triangle(ps[0], ps[1], ps[2]), W);
			triangles.emplace_back(Triangle(ps[0], ps[2], ps[3]), W);

			triangles.emplace_back(Triangle(ps[0], ps[4], ps[7]), W);
			triangles.emplace_back(Triangle(ps[3], ps[0], ps[7]), W);

			triangles.emplace_back(Triangle(ps[4], ps[5], ps[6]), W);
			triangles.emplace_back(Triangle(ps[6], ps[7], ps[4]), W);

			LambertianMaterial L(Vec3(3.0), Vec3(1.0));
			double s = 0.7;
			Vec3 light[] = {
				Vec3(-hsize * s, hsize * 0.99 , -hsize * s),
				Vec3(-hsize * s, hsize * 0.99, hsize * s),
				Vec3(hsize * s, hsize * 0.99, hsize * s),
				Vec3(hsize * s, hsize * 0.99, -hsize * s),
			};
			for (int i = 0; i < sizeof(light) / sizeof(light[0]); ++i) {
				light[i].z += z;
			}
			triangles.emplace_back(Triangle(light[0], light[1], light[2]), L);
			triangles.emplace_back(Triangle(light[0], light[2], light[3]), L);

			//sphere1.center = Vec3(0.0, 0.0, -3.0);
			//sphere1.radius = 0.5f;
			//sphereMat1 = LambertianMaterial(Vec3(), Vec3(0.75));
		}

		struct ConelTriangle {
			ConelTriangle() { }
			ConelTriangle(const Triangle &t, const LambertianMaterial &m) :triangle(t), mat(m){

			}

			Triangle triangle;
			LambertianMaterial mat;
		};
		std::vector<ConelTriangle> triangles;

		// Sphere sphere1;
		LambertianMaterial sphereMat1;

		bool intersect(const Ray &ray, LambertianMaterial *mat, Intersection *intersection, double tmin = std::numeric_limits<double>::max()) const {
			intersection->tmin = tmin;

			bool intersected = false;
			for (int i = 0; i < triangles.size(); ++i) {
				TriangleIntersection thisIntersection;
				if (intersect_triangle(ray, triangles[i].triangle, &thisIntersection, intersection->tmin)) {
					if (thisIntersection.tmin < intersection->tmin) {
						auto triangle = triangles[i];
						*mat = triangle.mat;
						*intersection = thisIntersection;
						intersected = true;
					}
				}
			}

			//Intersection sphere_intersection;
			//if (intersect_shere(ray, sphere1, &sphere_intersection, intersection->tmin)) {
			//	*mat = sphereMat1;
			//	*intersection = sphere_intersection;
			//}

			return intersected;
		}
	};

	inline Vec3 uniformHemisphere(PeseudoRandom &random) {
		Vec3 d;
		double lengthSquared;
		do {
			d.x = random.uniform(-1.0, 1.0);
			d.y = random.uniform(0.0, 1.0);
			d.z = random.uniform(-1.0, 1.0);
			lengthSquared = glm::length2(d);
		} while (lengthSquared < 0.000001 || 1.0 < lengthSquared);
		return d / glm::sqrt(lengthSquared);
	}

	Vec3 SampleCosineWeighted(PeseudoRandom &random) {
		double u1 = random.uniform();
		double u2 = random.uniform();
		double r = sqrt(u1);
		double phi = glm::two_pi<double>() * u2;
		return Vec3(r * glm::cos(phi), glm::sqrt(1.0 - u1), r * glm::sin(phi));
	}

	inline Vec3 radiance(const rt::ConelboxObject &conelbox, Ray ray, PeseudoRandom &random) {
		Vec3 Lo;
		Vec3 T(1.0);
		for (int i = 0; i < 20; ++i) {
			LambertianMaterial mat;
			Intersection intersection;
			if (conelbox.intersect(ray, &mat, &intersection)) {
				auto sample = SampleCosineWeighted(random);
				
				// p = cos(θ) / pi
				auto p = glm::dot(sample, Vec3(0.0, 1.0, 0.0)) * glm::one_over_pi<double>();

				auto wi = basisTransform(sample, intersection.normal);
				Lo += mat.Le * T;
				T *= mat.R * Vec3(glm::one_over_pi<double>()) * glm::dot(intersection.normal, wi) / p;
				ray = Ray(ray.o + ray.d * intersection.tmin + wi * 0.000001, wi);
			}
			else {
				break;
			}
		}
		return Lo;
	}

	inline void step(rt::Camera &camera, const rt::ConelboxObject &conelbox) {
		#pragma omp parallel for schedule(dynamic, 16)
		for (int y = 0; y < camera.film()->width(); ++y) {
			for (int x = 0; x < camera.film()->height(); ++x) {
				Xor &random = camera.film()->pixel(x, y)->random;
				double dx = random.uniform(-0.5, 0.5);
				double dy = random.uniform(-0.5, 0.5);
				auto ray = camera.generateRay(x + dx, y + dy);

				rt::Intersection intersection;
				rt::LambertianMaterial mat;
				camera.film()->add(x, y, radiance(conelbox, ray, random));
			}
		}
	}
}


inline ofVec3f toOf(rt::Vec3 v) {
	return ofVec3f(v.x, v.y, v.z);
}
inline ofPixels toOf(const rt::Camera &camera) {
	ofPixels pixels;
	pixels.allocate(camera.film()->width(), camera.film()->height(), OF_IMAGE_COLOR);
	uint8_t *dst = pixels.getPixels();

	double scale = 1.0;
	for (int y = 0; y < camera.film()->width(); ++y) {
		for (int x = 0; x < camera.film()->height(); ++x) {
			int index = y * camera.film()->height() + x;
			auto px = *camera.film()->pixel(x, y);
			auto L = px.color / (double)px.sample;
			dst[index * 3 + 0] = (uint8_t)glm::clamp(glm::pow(L.x * scale, 1.0 / 2.2) * 255.0, 0.0, 255.99999);
			dst[index * 3 + 1] = (uint8_t)glm::clamp(glm::pow(L.y * scale, 1.0 / 2.2) * 255.0, 0.0, 255.99999);
			dst[index * 3 + 2] = (uint8_t)glm::clamp(glm::pow(L.z * scale, 1.0 / 2.2) * 255.0, 0.0, 255.99999);
		}
	}
	return pixels;
}
//
//class GLRenderer {
//public:
//	void render(rt::Scene &scene, ofFbo &fbo) {
//		if (fbo.getWidth() != scene._cameraSetting._imageWidth ||
//			fbo.getHeight() != scene._cameraSetting._imageHeight) {
//			fbo.allocate(scene._cameraSetting._imageWidth, scene._cameraSetting._imageHeight);
//		}
//		rt::PinholeCamera camera(scene._cameraSetting);
//
//		fbo.begin();
//		ofClear(0);
//		ofEnableDepthTest();
//
//		glMatrixMode(GL_PROJECTION);
//		glLoadMatrixd(glm::value_ptr(camera._proj));
//		glMatrixMode(GL_MODELVIEW);
//		glLoadMatrixd(glm::value_ptr(camera._view));
//		//glLoadIdentity();
//
//		ofSetColor(255);
//		ofNoFill();
//		for (int i = 0; i < scene._sceneElements.size(); ++i) {
//			auto element = scene._sceneElements[i];
//			if (auto polygon = std::dynamic_pointer_cast<rt::PolygonSceneElement>(element)) {
//				for (int j = 0; j < polygon->_indices.size(); j += 3) {
//					const rt::Vec3 &v0 = polygon->_vertices[polygon->_indices[j + 0]];
//					const rt::Vec3 &v1 = polygon->_vertices[polygon->_indices[j + 1]];
//					const rt::Vec3 &v2 = polygon->_vertices[polygon->_indices[j + 2]];
//					ofDrawTriangle(toOf(v0), toOf(v1), toOf(v2));
//				}
//			}
//		}
//		ofFill();
//
//		fbo.end();
//	}
//};

//rt::Camera camera(256, 256);
//rt::ConelboxObject conelbox;

class OrenNayar {
public:
	double brdf(rt::Vec3 wi, rt::Vec3 wo, rt::Vec3 n) {
		double n_dot_wi = glm::dot(n, wi);
		double n_dot_wo = glm::dot(n, wo);
		double theta_i = glm::acos(n_dot_wi);
		double theta_o = glm::acos(n_dot_wo);
		double alpha = glm::max(theta_i, theta_o);
		double beta = glm::min(theta_i, theta_o);
		double A = 1.0 - 0.5 * (sigma * sigma / (sigma * sigma + 0.33));
		double B = 0.45 * (sigma * sigma / (sigma * sigma + 0.09));
		float c = glm::max(
			glm::dot(wo - n * glm::dot(wo, n), wi - n * glm::dot(wi, n)),
			0.0
		);
		return glm::one_over_pi<double>() * (A + (B * c * glm::sin(alpha) * glm::tan(beta)));
	}
	double sigma = 0.0;
};



//--------------------------------------------------------------
void ofApp::setup(){
	_imgui.setup();

	_camera.setNearClip(0.1f);
	_camera.setFarClip(100.0f);
	_camera.setDistance(5.0f);

	_scene = scene_conelbox();

	//rt::Vec3 N(0.0, 1.0, 0.0);
	//rt::Xor random;
	//int MonteCalroN = 10000;
	//OrenNayar oren;
	//oren.sigma = 0.5;

	//for (int i = 0; i < 100; ++i) {
	//	rt::Vec3 wi = uniformHemisphere(random);

	//	double M = 0.0;
	//	for (int j = 0; j < MonteCalroN; ++j) {
	//		rt::Vec3 wo = uniformHemisphere(random);
	//		double p = glm::one_over_two_pi<double>();
	//		double brdf = oren.brdf(wi, wo, N);
	//		M += brdf * glm::dot(wo, N) / p;
	//	}
	//	M /= MonteCalroN;
	//	printf("%.5f\n", M);
	//}
}

//--------------------------------------------------------------
void ofApp::update(){
	//rt::step(camera, conelbox);

	//if (ofGetFrameNum() % 30 == 0) {
	//	_image.setFromPixels(toOf(camera));
	//}
}

//--------------------------------------------------------------
void ofApp::draw(){
	ofEnableDepthTest();

	ofClear(0);
	_camera.begin();
	ofPushMatrix();
	ofRotateZ(90.0f);
	ofSetColor(255);
	ofDrawGridPlane(1.0f);
	ofPopMatrix();

	ofPushMatrix();
	ofDrawAxis(50);
	ofPopMatrix();

	auto camera = _scene->_camera;

	for (int y = 0; y < _scene->_cameraSetting._imageHeight; y += 64) {
		for (int x = 0; x < _scene->_cameraSetting._imageWidth; x += 64) {
			auto ray = camera->generateRay(x, y);
			ofDrawLine(toOf(ray.o), toOf(ray.o) + toOf(ray.d) * 3);
		}
	}
	{
		auto ray = _scene->_camera->generateRay(0, _scene->_cameraSetting._imageHeight);
		ofDrawLine(toOf(ray.o), toOf(ray.o) + toOf(ray.d) * 3);
	}
	{
		auto ray = camera->generateRay(_scene->_cameraSetting._imageWidth - 1, 0);
		ofDrawLine(toOf(ray.o), toOf(ray.o) + toOf(ray.d) * 3);
	}
	{
		auto ray = camera->generateRay(_scene->_cameraSetting._imageWidth, _scene->_cameraSetting._imageHeight - 1);
		ofDrawLine(toOf(ray.o), toOf(ray.o) + toOf(ray.d) * 3);
	}

	ofSetColor(255);
	ofNoFill();
	for (int i = 0; i < _scene->_sceneElements.size(); ++i) {
		auto element = _scene->_sceneElements[i];
		if (auto polygon = std::dynamic_pointer_cast<rt::PolygonSceneElement>(element)) {
			for (int j = 0; j < polygon->_indices.size(); j += 3) {
				const rt::Vec3 &v0 = polygon->_vertices[polygon->_indices[j + 0]];
				const rt::Vec3 &v1 = polygon->_vertices[polygon->_indices[j + 1]];
				const rt::Vec3 &v2 = polygon->_vertices[polygon->_indices[j + 2]];
				ofDrawTriangle(toOf(v0), toOf(v1), toOf(v2));
			}
		}
	}
	ofFill();

	rt::Xor random;
	for (int i = 0; i < 1000; ++i) {
		rt::Vec3 p;
		rt::Vec3 n;
		rt::Vec3 L;
		_scene->sampleEmissive(&p, &n, &L, &random);
		ofDrawSphere(toOf(p), 0.01f);
		ofDrawLine(toOf(p), toOf(p) + toOf(n) * 0.1f);
	}

	/*
	for (int y = 0; y < 256; y += 32) {
		for (int x = 0; x < 256; x += 32) {
			auto ray = camera.generateRay(x, y);
			ofDrawLine(ofPoint(), ofPoint(ray.d.x, ray.d.y, ray.d.z) * 3);
		}
	}

	for (int i = 0; i < conelbox.triangles.size(); ++i) {
		auto tri = conelbox.triangles[i];
		ofSetColor(tri.mat.R.x * 255, tri.mat.R.y * 255, tri.mat.R.z * 255);
		ofDrawTriangle(toOf(tri.triangle.v[0]), toOf(tri.triangle.v[1]), toOf(tri.triangle.v[2]));
	}
	*/

	_camera.end();

	//GLRenderer glRender;
	//glRender.render(scene, _previewFbo);

	//ofSetColor(255);
	//ofDisableAlphaBlending();
	//ofDisableDepthTest();
	//_previewFbo.getTexture().drawSubsection(
	//	0, 0, _previewFbo.getWidth(), _previewFbo.getHeight(),
	//	0, _previewFbo.getHeight(), _previewFbo.getWidth(), -_previewFbo.getHeight());
	//ofEnableAlphaBlending();

	ofDisableDepthTest();
	ofSetColor(255);

	_image.draw(0, 0);

	_imgui.begin();

	ImGui::PushStyleColor(ImGuiCol_WindowBg, ofVec4f(0.2f, 0.2f, 0.5f, 0.5f));
	ImGui::SetNextWindowPos(ofVec2f(10, 30), ImGuiSetCond_Once);
	ImGui::SetNextWindowSize(ofVec2f(500, ofGetHeight() * 0.8), ImGuiSetCond_Once);

	ImGui::Begin("Config Panel");
	ImGui::Text("fps: %.2f", ofGetFrameRate());
	ImGui::SliderFloat("_cameraZ", &_cameraZ, 0.1f, 3.0f);
	auto wp = ImGui::GetWindowPos();
	auto ws = ImGui::GetWindowSize();
	ofRectangle win(wp.x, wp.y, ws.x, ws.y);

	ImGui::End();
	ImGui::PopStyleColor();

	_imgui.end();

	if (win.inside(ofGetMouseX(), ofGetMouseY())) {
		_camera.disableMouseInput();
	} else {
		_camera.enableMouseInput();
	}
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key){
	if (key == 's') {
		char name[] = "";
		sprintf(name, "image.png");
		_image.save(name);
	}
}

//--------------------------------------------------------------
void ofApp::keyReleased(int key){

}

//--------------------------------------------------------------
void ofApp::mouseMoved(int x, int y ){

}

//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mouseEntered(int x, int y){

}

//--------------------------------------------------------------
void ofApp::mouseExited(int x, int y){

}

//--------------------------------------------------------------
void ofApp::windowResized(int w, int h){

}

//--------------------------------------------------------------
void ofApp::gotMessage(ofMessage msg){

}

//--------------------------------------------------------------
void ofApp::dragEvent(ofDragInfo dragInfo){ 

}
