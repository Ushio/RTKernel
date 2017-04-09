#pragma once

#include <atomic>
#include <memory>

#include <glm/glm.hpp>
#include <glm/ext.hpp>
#include <boost/variant.hpp>

/*
約束

光源はlambertianのみ
*/

namespace rt {
	typedef glm::dvec2 Vec2;
	typedef glm::dvec3 Vec3;
	typedef glm::dvec4 Vec4;
	typedef glm::dmat3 Mat3;
	typedef glm::dmat4 Mat4;

	struct Ray {
		Ray() {}
		Ray(Vec3 orig, Vec3 dir) :o(orig), d(dir) {}
		Vec3 o;
		Vec3 d;
	};

	struct PeseudoRandom {
		virtual ~PeseudoRandom() {}

		virtual uint32_t generate() = 0;
		virtual double uniform() = 0;
		virtual double uniform(double a, double b) = 0;
	};
	struct Xor : public PeseudoRandom {
		Xor() {

		}
		Xor(uint32_t seed) {
			_y = std::max(seed, 1u);
		}

		// 0 <= x <= 0x7FFFFFFF
		uint32_t generate() {
			_y = _y ^ (_y << 13); _y = _y ^ (_y >> 17);
			uint32_t value = _y = _y ^ (_y << 5); // 1 ~ 0xFFFFFFFF(4294967295
			return value >> 1;
		}
		// 0.0 <= x < 1.0
		double uniform() {
			return double(generate()) / double(0x80000000);
		}
		double uniform(double a, double b) {
			return a + (b - a) * double(uniform());
		}
	public:
		uint32_t _y = 2463534242;
	};
	struct AtomicXor : public PeseudoRandom {
		AtomicXor() {

		}
		AtomicXor(uint32_t seed) {
			_y = std::max(seed, 1u);
		}

		// 0 <= x <= 0x7FFFFFFF
		uint32_t generate() {
			uint32_t expected = _y.load();
			uint32_t desired;
			do {
				int y = expected;
				y = y ^ (y << 13);
				y = y ^ (y >> 17);
				y = y ^ (y << 5);
				desired = y;
			} while (!_y.compare_exchange_weak(expected, desired));
			uint32_t value = desired; // 1 ~ 0xFFFFFFFF(4294967295
			return desired >> 1;
		}
		// 0.0 <= x < 1.0
		double uniform() {
			return double(generate()) / double(0x80000000);
		}
		double uniform(double a, double b) {
			return a + (b - a) * double(uniform());
		}
	public:
		std::atomic<uint32_t> _y = 2463534242;
	};

	struct CameraSetting {
		double _fov = glm::radians(45.0);

		Vec3 _eye = Vec3(0.0, 0.0, 1.0);
		Vec3 _lookat = Vec3(0.0, 0.0, 0.0);
		Vec3 _up = Vec3(0.0, 1.0, 0.0);

		int _imageWidth = 1024;
		int _imageHeight = 768;
	};

	inline Vec3 mul3x4(const Mat4 &m, const Vec3 v) {
		return Vec3(m * Vec4(v, 1.0));
	}

	class PinholeCamera {
	public:
		PinholeCamera(CameraSetting setting) {
			_setting = setting;

			double w = _setting._imageWidth;
			double h = _setting._imageHeight;
			_proj = glm::perspectiveFov(_setting._fov, w, h, 1.0, 100.0);

			auto vp = Vec4(0.0, 0.0, w, h);
			_origin = _setting._eye;
			_UR = glm::unProject(Vec3(0.0, h - 1, 0.0), Mat4(), _proj, vp);
			auto r = glm::unProject(Vec3(w - 1, h - 1, 0.0), Mat4(), _proj, vp);
			auto b = glm::unProject(Vec3(0.0, 0.0, 0.0), Mat4(), _proj, vp);
			_dirRright = r - _UR;
			_dirBottom = b - _UR;

			_view = glm::lookAt(_setting._eye, _setting._lookat, _setting._up);
			_ray_p_transform = glm::inverse(_view);
			_ray_d_transform = Mat3(_ray_p_transform);// glm::transpose(Mat3(_view));
		}

		Ray generateRay(double filmx, double filmy) const {
			Ray ray;
			ray.d = glm::normalize(_UR + _dirRright * (filmx / _setting._imageWidth) + _dirBottom * (filmy / _setting._imageHeight));
			ray.o = mul3x4(_ray_p_transform, ray.o);
			ray.d = _ray_d_transform * ray.d;
			return ray;
		}

		CameraSetting _setting;

		Mat4 _proj;
		Mat4 _view;
		Mat4 _ray_p_transform;
		Mat3 _ray_d_transform;

		Vec3 _origin;
		Vec3 _UR;
		Vec3 _dirRright;
		Vec3 _dirBottom;
	};

	class LambertianMaterial {
	public:
		LambertianMaterial() {}
		LambertianMaterial(Vec3 e, Vec3 r) : Le(e), R(r) {}
		Vec3 Le;
		Vec3 R;
	};
	class SpecularMaterial {
	public:
	};

	class Material {
	public:
		Material() {}
		template <class T>
		Material(const T& value):_content(value){

		}

		template <class T>
		void operator=(const T &material) {
			_content = material;
		}

		template <class T>
		bool isType() const {
			return _content.type() == typeid(T);
		}
		template <class T>
		T get() const {
			return boost::get<T>(_content);
		}

		boost::variant<LambertianMaterial, SpecularMaterial> _content;
	};

	struct Intersection {
		double tmin = std::numeric_limits<double>::max();
		Vec3 normal;
	};
	struct TriangleIntersection : public Intersection {
		Vec2 uv;
	};

	inline Vec3 triangle_normal(const Vec3 &v0, const Vec3 &v1, const Vec3 &v2, bool isback) {
		Vec3 e1 = v1 - v0;
		Vec3 e2 = v2 - v0;
		return glm::normalize(isback ? glm::cross(e2, e1) : glm::cross(e1, e2));
	}

	inline bool intersect_triangle(const Ray &ray, const Vec3 &v0, const Vec3 &v1, const Vec3 &v2, TriangleIntersection *intersection, double tmin = std::numeric_limits<double>::max()) {
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
		intersection->normal = triangle_normal(v0, v1, v2, isback);
		intersection->uv = Vec2(baryPosition.x, baryPosition.y);
		return true;
	}
	inline double triangle_area(const Vec3 &p0, const Vec3 &p1, const Vec3 &p2) {
		auto va = p0 - p1;
		auto vb = p2 - p1;
		return glm::length(glm::cross(va, vb)) * 0.5;
	}

	inline Vec3 uniform_on_triangle(PeseudoRandom *random, const Vec3 &p0, const Vec3 &p1, const Vec3 &p2) {
		double eps1 = random->uniform();
		double eps2 = random->uniform();

		double sqrt_r1 = glm::sqrt(eps1);
		Vec3 p =
			(1.0 - sqrt_r1) * p0
			+ sqrt_r1 * (1.0 - eps2) * p1
			+ sqrt_r1 * eps2 * p2;
		return p;
	}

	struct Sphere {
		Vec3 center;
		double radius = 1.0;
	};

	inline bool intersect_shere(const Ray &ray, const Sphere &s, Intersection *intersection, double tmin = std::numeric_limits<double>::max()) {
		Vec3 m = ray.o - s.center;
		double b = glm::dot(m, ray.d);
		double radius_squared = s.radius * s.radius;
		double c = glm::dot(m, m) - radius_squared;
		if (c > 0.0 && b > 0.0) {
			return false;
		}
		double discr = b * b - c;
		if (discr < 0.0) {
			return false;
		}

		double sqrt_discr = glm::sqrt(discr);
		double tmin_near = -b - sqrt_discr;
		if (0.0 < tmin_near) {
			// back = false
			auto p = ray.o + ray.d * tmin_near;
			intersection->tmin = tmin_near;
			intersection->normal = glm::normalize(p - s.center);
			return true;
		}
		double tmin_far = -b + sqrt_discr;
		if (0.0 < tmin_far) {
			// back = true
			auto p = ray.o + ray.d * tmin_far;
			intersection->tmin = tmin_far;
			intersection->normal = glm::normalize(s.center - p);
			return true;
		}
		return false;
	}

	// 面積が均一になるようなサンプリング
	class AreaUniformSampler {
	public:
		AreaUniformSampler(const std::vector<double> &areas) {
			double area = 0.0;
			for (int i = 0; i < areas.size(); ++i) {
				area += areas[i];
				_cumulativeAreas.push_back(area);
			}
			_area = area;
		}
		int sample(PeseudoRandom *random) const {
			double area_at = random->uniform(0.0, _area);
			auto it = std::upper_bound(_cumulativeAreas.begin(), _cumulativeAreas.end(), area_at);
			std::size_t index = std::distance(_cumulativeAreas.begin(), it);
			index = std::min(index, _cumulativeAreas.size() - 1);
			return (int)index;
		}
		double _area = 0.0;
		std::vector<double> _areas;
		std::vector<double> _cumulativeAreas;
	};

	class SceneElement {
	public:
		virtual ~SceneElement() {}

		virtual bool intersect(const Ray &ray, Material *mat, Intersection *intersection, double tmin) const = 0;
		
		virtual double emissiveArea() const = 0;
		virtual void sampleEmissive(Vec3 *p, Vec3 *n, Vec3 *emissiveRadiance, PeseudoRandom *random) const = 0;
	};

	// 最適化しないといけないなら仕方ないね
	class PolygonSceneElement : public SceneElement {
	public:
		PolygonSceneElement(const std::vector<Vec3> &vertices, std::vector<int> const &indices, Material material, bool backEmissive)
		:_vertices(vertices)
		,_indices(indices)
		,_material(material)
		,_backEmissive(backEmissive) {
			_triangleCumulativeAreas.resize(_indices.size() / 3);
			double area = 0.0;
			for (int i = 0; i < _indices.size(); i += 3) {
				const Vec3 &v0 = _vertices[_indices[i + 0]];
				const Vec3 &v1 = _vertices[_indices[i + 1]];
				const Vec3 &v2 = _vertices[_indices[i + 2]];

				area += triangle_area(v0, v1, v2);
				_triangleCumulativeAreas[i / 3] = area;
			}
			_area = area;
		}

		bool intersect(const Ray &ray, Material *mat, Intersection *intersection, double tmin) const override {
			intersection->tmin = tmin;

			bool intersected = false;
			for (int i = 0; i < _indices.size(); i += 3) {
				const Vec3 &v0 = _vertices[_indices[i + 0]];
				const Vec3 &v1 = _vertices[_indices[i + 1]];
				const Vec3 &v2 = _vertices[_indices[i + 2]];
				TriangleIntersection thisIntersection;
				if (intersect_triangle(ray, v0, v1, v2, &thisIntersection, intersection->tmin)) {
					if (thisIntersection.tmin < intersection->tmin) {
						*mat = _material;
						*intersection = thisIntersection;
						intersected = true;
					}
				}
			}
			return intersected;
		}

		double emissiveArea() const override {
			if (_material.isType<LambertianMaterial>()) {
				if (_backEmissive) {
					return _area * 2.0;
				}
				return _area;
			}
			return 0.0;
		}

		void sampleEmissive(Vec3 *p, Vec3 *n, Vec3 *emissiveRadiance, PeseudoRandom *random) const override {
			double area_at = random->uniform(0.0, _area);
			auto it = std::upper_bound(_triangleCumulativeAreas.begin(), _triangleCumulativeAreas.end(), area_at);
			std::size_t index = std::distance(_triangleCumulativeAreas.begin(), it);
			index = std::min(index, _triangleCumulativeAreas.size() - 1);
			
			auto p0 = _vertices[_indices[index * 3]];
			auto p1 = _vertices[_indices[index * 3 + 1]];
			auto p2 = _vertices[_indices[index * 3 + 2]];
			*p = uniform_on_triangle(random,
				p0,
				p1,
				p2
			);
			if (_backEmissive) {
				*n = triangle_normal(p0, p1, p2, random->uniform() < 0.5);
			}
			else {
				*n = triangle_normal(p0, p1, p2, false);
			}

			if (_material.isType<LambertianMaterial>()) {
				*emissiveRadiance = _material.get<LambertianMaterial>().Le;
			}
			else {
				assert(0);
			}
		}

		std::vector<Vec3> _vertices;
		std::vector<int> _indices;

		Material _material;

		// 両面の対応はすべてSceneElementの中で対応可能
		bool _backEmissive = false;
		
		// ライトサンプル用
		std::vector<double> _triangleCumulativeAreas;
		double _area = 0.0;
	};
	class SphereSceneElement : public SceneElement {
	public:
		Material _material;
		std::vector<Vec3> _vertices;
		std::vector<int> _indices;
	};

	class Scene {
	public:
		Scene(const CameraSetting &cameraSetting, const std::vector<std::shared_ptr<SceneElement>> &sceneElements)
		:_cameraSetting(cameraSetting)
		,_sceneElements(sceneElements)
		,_camera(new PinholeCamera(cameraSetting)) {
			for (int i = 0; i < _sceneElements.size(); ++i) {
				if (glm::epsilon<double>() <= _sceneElements[i]->emissiveArea()) {
					_emissiveElements.push_back(_sceneElements[i]);
				}
			}

			std::vector<double> areas;
			double area = 0.0;
			for (int i = 0; i < _emissiveElements.size(); ++i) {
				areas.push_back(_emissiveElements[i]->emissiveArea());
			}
			_areaUniformSampler = std::unique_ptr<AreaUniformSampler>(new AreaUniformSampler(areas));
		}

		bool intersect(const Ray &ray, Material *mat, Intersection *intersection, double tmin) const {
			intersection->tmin = tmin;

			bool intersected = false;
			for (int i = 0; i < _sceneElements.size(); ++i) {
				Intersection thisIntersection;
				Material thisMaterial;
				if (_sceneElements[i]->intersect(ray, &thisMaterial, &thisIntersection, intersection->tmin)) {
					if (thisIntersection.tmin < intersection->tmin) {
						*mat = thisMaterial;
						*intersection = thisIntersection;
						intersected = true;
					}
				}
			}
			return intersected;
		}

		void sampleEmissive(Vec3 *p, Vec3 *n, Vec3 *emissiveRadiance, PeseudoRandom *random) const {
			int index = _areaUniformSampler->sample(random);
			_emissiveElements[index]->sampleEmissive(p, n, emissiveRadiance, random);
		}

		CameraSetting _cameraSetting;
		std::shared_ptr<PinholeCamera> _camera;
		std::vector<std::shared_ptr<SceneElement>> _sceneElements;

		// ライトサンプル用
		std::vector<std::shared_ptr<SceneElement>> _emissiveElements;
		std::unique_ptr<AreaUniformSampler> _areaUniformSampler;
	};

	class Image {
	public:

	};
}
