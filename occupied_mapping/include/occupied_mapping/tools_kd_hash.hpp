#pragma once
#include <iostream>
#include <unordered_map>
#include <functional>

#if 1

#define HASH_PRIME 116101
#define MAX_N 201326611

class Hash_3D_key {
public:
	int64_t x, y, z;

	Hash_3D_key(int64_t vx = 0, int64_t vy = 0, int64_t vz = 0) :
		x(vx), y(vy), z(vz) {}

	bool operator==(const Hash_3D_key &other) const { return (x == other.x && y == other.y && z == other.z); }
};

// Hash value
namespace std {
	template <>
	struct hash<Hash_3D_key> {
		int64_t operator()(const Hash_3D_key &s) const {
			using std::hash;
			using std::size_t;
			return ((((s.z) * HASH_PRIME) % MAX_N + (s.y)) * HASH_PRIME) % MAX_N + (s.x);
		}
	};
} // namespace std

template <typename data_type = float, typename T = void *>
struct Hash_map_3d {
	// const double N = Common_tools::Hash_key_factor< data_type >::factor;
	double N = 1;
	using hash_3d_T = std::unordered_map<Hash_3D_key, T>;
	hash_3d_T m_map_3d_hash_map;
	Hash_map_3d() {
		if (std::is_same<float, data_type>::value) {
			N = 1e5;
		} else if (std::is_same<double, data_type>::value) {
			N = 1e7;
		}
	}
	T *get_data(const data_type &x, const data_type &y, const data_type &z) {
		typename hash_3d_T::iterator it = m_map_3d_hash_map.find(Hash_3D_key(x * N, y * N, z * N));
		if (it == m_map_3d_hash_map.end()) {
			// cout << "Get data not found, return nullptr." << endl;
			return nullptr;
		} else {
			return &(it->second);
		}
	}
	void reserve(long size) {
		m_map_3d_hash_map.reserve(size);
	}

	void insert(const data_type &x, const data_type &y, const data_type &z, const T &target) { m_map_3d_hash_map[Hash_3D_key(x * N, y * N, z * N)] = target; }

	void erase(const data_type &x, const data_type &y, const data_type &z){ m_map_3d_hash_map.erase(Hash_3D_key(x * N, y * N, z * N)); }

	bool if_exist(const data_type &x, const data_type &y, const data_type &z) {
		if (m_map_3d_hash_map.find(Hash_3D_key(x * N, y * N, z * N)) == m_map_3d_hash_map.end()) {
			return false;
		}
		return true;
	}

	void clear() { m_map_3d_hash_map.clear(); }

	//将哈希表所有点装入 all_data返回
	void all_data(std::vector<T> &all_data) {
		all_data.clear();
		all_data.reserve(m_map_3d_hash_map.size());
		for (auto it = m_map_3d_hash_map.begin(); it != m_map_3d_hash_map.end(); it++) {
			all_data.push_back(it->second);
		}
	}
	//删除边界外的点
	void erase_data_out_of_range(const data_type &x_min, const data_type &x_max, const data_type &y_min, const data_type &y_max, const data_type &z_min, const data_type &z_max) {
		for (auto it = m_map_3d_hash_map.begin(); it != m_map_3d_hash_map.end();) {
			if (it->first.x < x_min * N || it->first.x > x_max * N || it->first.y < y_min * N || it->first.y > y_max * N || it->first.z < z_min * N || it->first.z > z_max * N) {
				it = m_map_3d_hash_map.erase(it);
			} else {
				it++;
			}
		}
	}

	void erase_data(const data_type &x, const data_type &y, const data_type &z) {
		m_map_3d_hash_map.erase(Hash_3D_key(x * N, y * N, z * N));
	}

	size_t get_memory() {
		return (m_map_3d_hash_map.size() * (sizeof(T) + sizeof(void *)) + m_map_3d_hash_map.bucket_count() * (sizeof(void *) + sizeof(size_t))) * 2;
	}

	int total_size() { return m_map_3d_hash_map.size(); }
};

#else
template <typename data_type = float, typename T = void *>
struct Hash_map_3d {
	using hash_3d_T = std::unordered_map<data_type, std::unordered_map<data_type, std::unordered_map<data_type, T>>>;
	hash_3d_T m_map_3d_hash_map;
	void insert(const data_type &x, const data_type &y, const data_type &z, const T &target) { m_map_3d_hash_map[x][y][z] = target; }

	int if_exist(const data_type &x, const data_type &y, const data_type &z) {
		if (m_map_3d_hash_map.find(x) == m_map_3d_hash_map.end()) {
			return 0;
		} else if (m_map_3d_hash_map[x].find(y) == m_map_3d_hash_map[x].end()) {
			return 0;
		} else if (m_map_3d_hash_map[x][y].find(z) == m_map_3d_hash_map[x][y].end()) {
			return 0;
		}
		return 1;
	}

	void clear() { m_map_3d_hash_map.clear(); }

	int total_size() {
		int count = 0;
		for (auto it : m_map_3d_hash_map) {
			for (auto it_it : it.second) {
				for (auto it_it_it : it_it.second) {
					count++;
				}
			}
		}
		return count;
	}
};
#endif

template <typename data_type = float, typename T = void *>
struct Hash_map_2d {
	using hash_2d_T = std::unordered_map<data_type, std::unordered_map<data_type, T>>;
	// using hash_2d_it = typename std::unordered_map<data_type, std::unordered_map<data_type, T> >::iterator ;
	// using hash_2d_it_it = typename std::unordered_map<data_type, T>::iterator ;

	hash_2d_T m_map_2d_hash_map;
	void insert(const data_type &x, const data_type &y, const T &target) { m_map_2d_hash_map[x][y] = target; }

	int if_exist(const data_type &x, const data_type &y) {
		if (m_map_2d_hash_map.find(x) == m_map_2d_hash_map.end()) {
			return 0;
		} else if (m_map_2d_hash_map[x].find(y) == m_map_2d_hash_map[x].end()) {
			return 0;
		}

		return 1;
	}

	T get_data(const data_type &x, const data_type &y) {
		return m_map_2d_hash_map[x][y];
	}

	void clear() { m_map_2d_hash_map.clear(); }

	int total_size() {
		int count = 0;
		// for(hash_2d_it it =  m_map_2d_hash_map.begin(); it != m_map_2d_hash_map.end(); it++)
		for (auto it : m_map_2d_hash_map) {
			for (auto it_it : it.second) {
				count++;
			}
		}
		return count;
	}
};
