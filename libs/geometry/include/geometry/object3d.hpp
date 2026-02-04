/// @file object3d.hpp
/// @brief 3D object representation with OBJ file I/O.
///
/// Provides Triangle and Object3D structures for representing 3D meshes,
/// along with functions for reading and writing Wavefront OBJ files.

#pragma once

#include <geometry/line3.hpp>

#include <map>
#include <sstream>
// Removed Boost: provide local helpers instead
#include <algorithm>
#include <cctype>
#include <stdexcept>
// #include <boost/filesystem/fstream.hpp>     // <--- Opengl has trouble if included

#include <fstream>
#include <string>

namespace lsfm {

/// @brief Trim whitespace from both ends of a string in place.
/// @param s String to trim.
inline void trim_inplace(std::string& s) {
  auto not_space = [](unsigned char ch) { return !std::isspace(ch); };
  s.erase(s.begin(), std::find_if(s.begin(), s.end(), not_space));
  s.erase(std::find_if(s.rbegin(), s.rend(), not_space).base(), s.end());
}

/// @brief Triangle face with vertex indices and texture coordinates.
///
/// Represents a triangle in a 3D mesh by storing indices into vertex,
/// normal, and texture coordinate arrays.
struct Triangle {
  /// @brief Construct triangle with vertex and texture indices.
  /// @param va First vertex index (default 0).
  /// @param vb Second vertex index (default 0).
  /// @param vc Third vertex index (default 0).
  /// @param vn Normal index (default 0).
  /// @param texa First texture coordinate index (default 0).
  /// @param texb Second texture coordinate index (default 0).
  /// @param texc Third texture coordinate index (default 0).
  Triangle(std::size_t va = 0,
           std::size_t vb = 0,
           std::size_t vc = 0,
           std::size_t vn = 0,
           std::size_t texa = 0,
           std::size_t texb = 0,
           std::size_t texc = 0)
      : a(va), b(vb), c(vc), normal(vn), ta(texa), tb(texb), tc(texc) {}

  std::size_t a, b, c, normal;  ///< Vertex and normal indices.
  std::size_t ta, tb, tc;       ///< Texture coordinate indices.
};

/// @brief 3D object with mesh data and OBJ file I/O.
///
/// Represents a 3D mesh consisting of vertices, normals, triangular faces,
/// and texture coordinates. Supports reading and writing Wavefront OBJ format.
/// @tparam FT Floating-point type.
template <class FT>
struct Object3D {
  typedef std::vector<Vec3<FT>> VertexList;        ///< List of 3D vertices.
  typedef std::vector<Vec3<FT>> NormalList;        ///< List of normal vectors.
  typedef std::vector<Triangle> TriangleList;      ///< List of triangle faces.
  typedef std::vector<Vec2<FT>> TexCoordList;      ///< List of texture coordinates.
  typedef std::vector<LineSegment3<FT>> EdgeList;  ///< List of 3D line segment edges.

  VertexList points;           ///< Mesh vertices.
  NormalList normals;          ///< Vertex normals.
  TriangleList triangles;      ///< Triangle faces.
  TexCoordList textureCoords;  ///< Texture coordinates.
  std::string name;            ///< Object name.

  /// @brief Default constructor.
  Object3D() {}

  /// @brief Construct with name, vertices, triangles, and texture coordinates.
  /// @param n Object name.
  /// @param p Vertex list.
  /// @param tri Triangle list.
  /// @param texC Texture coordinate list.
  Object3D(const std::string& n, const VertexList& p, const TriangleList& tri, const TexCoordList& texC)
      : name(n), points(p), triangles(tri), textureCoords(texC) {}

  /// @brief Construct from input stream.
  /// @param in Input stream containing OBJ data.
  Object3D(const std::istream& in) { read(in); }

  /// @brief Read single object from OBJ stream.
  ///
  /// Parses OBJ format including vertices (v), normals (vn), texture
  /// coordinates (vt), and faces (f). Updates offset counts for multi-object files.
  /// @param in Input stream.
  /// @param pointCount Running vertex count offset.
  /// @param normalCount Running normal count offset.
  /// @param textureCount Running texture coordinate count offset.
  void read(std::istream& in, int& pointCount, int& normalCount, int& textureCount) {
    name.clear();
    points.clear();
    normals.clear();
    triangles.clear();
    textureCoords.clear();
    int pointOffset = pointCount;
    int normalOffset = normalCount;
    int textureOffset = textureCount;

    std::string line;
    std::istream::pos_type g = in.tellg();
    while (std::getline(in, line)) {
      trim_inplace(line);
      if (line.size() < 3) {
        g = in.tellg();
        continue;
      }
      std::istringstream sin(line);
      std::string first;
      sin >> first;
      if (first == "o") {
        if (name.size()) {
          in.seekg(g);
          break;
        } else
          sin >> name;
      } else if (first == "v") {
        Vec3<FT> p;
        sin >> p.x() >> p.y() >> p.z();
        points.push_back(p);
        ++pointCount;
      } else if (first == "vn") {
        Vec3<FT> vn;
        sin >> vn.x() >> vn.y() >> vn.z();
        normals.push_back(vn);
        ++normalCount;
      } else if (first == "vt") {
        Vec2<FT> vt;
        sin >> vt.x() >> vt.y();
        textureCoords.push_back(vt);
        ++textureCount;
      } else if (first == "f") {
        std::string a, b, c, vn;
        std::size_t ta, tb, tc;
        sin >> a >> b >> c;
        std::size_t idx = a.find_last_of("/");
        if (idx != std::string::npos) vn = a.substr(idx + 1, std::string::npos);

        idx = a.find_first_of("/");
        if (idx != std::string::npos) {
          std::size_t idx2 = a.find_last_of("/");
          if (idx != idx2 && idx + 1 != idx2)
            ta = static_cast<std::size_t>(std::stoul(a.substr(idx + 1, idx2 - (idx + 1)))) - 1 - textureOffset;
          else
            ta = 0;
          a = a.substr(0, idx);
        }

        idx = b.find_first_of("/");
        if (idx != std::string::npos) {
          std::size_t idx2 = b.find_last_of("/");
          if (idx != idx2 && idx + 1 != idx2)
            tb = static_cast<std::size_t>(std::stoul(b.substr(idx + 1, idx2 - (idx + 1)))) - 1 - textureOffset;
          else
            tb = 0;
          b = b.substr(0, idx);
        }
        idx = c.find_first_of("/");
        if (idx != std::string::npos) {
          std::size_t idx2 = c.find_last_of("/");
          if (idx != idx2 && idx + 1 != idx2)
            tc = static_cast<std::size_t>(std::stoul(c.substr(idx + 1, idx2 - (idx + 1)))) - 1 - textureOffset;
          else
            tc = 0;
          c = c.substr(0, idx);
        }

        if (vn.empty()) {
          // compute normal from triangle
          triangles.push_back(Triangle(static_cast<std::size_t>(std::stoul(a)) - 1 - pointOffset,
                                       static_cast<std::size_t>(std::stoul(b)) - 1 - pointOffset,
                                       static_cast<std::size_t>(std::stoul(c)) - 1 - pointOffset, normals.size()));
          Triangle& tri = triangles.back();
          Vec3<FT> v1 = points[tri.b] - points[tri.a], v2 = points[tri.c] - points[tri.a];
          normals.push_back(v1.cross(v2));
          normals.back().normalize();
        } else
          triangles.push_back(Triangle(static_cast<std::size_t>(std::stoul(a)) - 1 - pointOffset,
                                       static_cast<std::size_t>(std::stoul(b)) - 1 - pointOffset,
                                       static_cast<std::size_t>(std::stoul(c)) - 1 - pointOffset,
                                       static_cast<std::size_t>(std::stoul(vn)) - 1 - normalOffset, ta, tb, tc));
      }
      g = in.tellg();
    }
  }

  /// @brief Write object to OBJ stream.
  /// @param out Output stream.
  void write(std::ostream& out) const {
    out << "o " << name << std::endl;
    for_each(points.begin(), points.end(),
             [&](const Vec3<FT>& p) { out << "v " << p.x() << " " << p.y() << " " << p.z() << std::endl; });

    for_each(triangles.begin(), triangles.end(),
             [&](const Triangle& t) { out << "f " << t.a + 1 << " " << t.b + 1 << " " << t.c + 1 << std::endl; });
  }

  /// @brief Get all edges including duplicates.
  ///
  /// Extracts all three edges from each triangle without removing duplicates.
  /// @return List of all triangle edges.
  EdgeList edgesAll() const {
    EdgeList ret;
    for_each(triangles.begin(), triangles.end(), [&](const Triangle& t) {
      ret.push_back(LineSegment3<FT>(points[t.a], points[t.b]));
      ret.push_back(LineSegment3<FT>(points[t.b], points[t.c]));
      ret.push_back(LineSegment3<FT>(points[t.c], points[t.a]));
    });
    return ret;
  }

  /// @brief Get unique edges excluding coplanar duplicates.
  ///
  /// Extracts edges from triangles, removing duplicates that lie on the same
  /// plane (internal mesh edges). Keeps silhouette/boundary edges.
  /// @return List of unique boundary edges.
  EdgeList edges() const {
    typedef std::map<std::size_t, Vec3<FT>> InternMap;
    typedef typename InternMap::iterator InternMapIter;
    typedef std::map<std::size_t, InternMap> MyMap;
    typedef typename MyMap::iterator MyMapIter;
    MyMap m;
    EdgeList ret;

    auto addEdge = [&](size_t a, size_t b, const Vec3<FT>& n) {
      MyMapIter f = m.find(a);
      InternMapIter f2;
      if (f == m.end()) {
        f = m.find(b);

        // nothing found
        if (f == m.end()) {
          m[a] = InternMap();
          m[a][b] = n;
          return;
        }

        f2 = f->second.find(a);
        // pair not found
        if (f2 == f->second.end()) {
          f->second.insert(typename InternMap::value_type(b, n));
          return;
        }
      } else {
        f2 = f->second.find(b);
        // pair not found
        if (f2 == f->second.end()) {
          f->second.insert(typename InternMap::value_type(a, n));
          return;
        }
      }

      // found duplicate -> check if we have to remove edge
      Vec3<FT> nd = n - f2->second;
      // same normal, remove from list
      if (std::fabs(nd.x() + nd.y() + nd.z()) < LIMITS<FT>::tau()) {
        if (f->second.size() > 1)
          f->second.erase(f2);
        else
          m.erase(f);
      }
      // else just keep existing edge in list
    };

    // detect multiple edges, sort them out and remove edges on same plane
    for_each(triangles.begin(), triangles.end(), [&](const Triangle& t) {
      Vec3<FT> n = (points[t.b] - points[t.a]).cross(points[t.c] - points[t.b]);
      addEdge(t.a, t.b, n);
      addEdge(t.b, t.c, n);
      addEdge(t.c, t.a, n);
    });

    // write edges to list
    for_each(m.begin(), m.end(), [&](const typename MyMap::value_type& p1) {
      for_each(p1.second.begin(), p1.second.end(), [&](const typename InternMap::value_type& p2) {
        ret.push_back(LineSegment3<FT>(points[p1.first], points[p2.first]));
      });
    });
    return ret;
  }


  /// @brief Find similar (nearly coincident) vertices.
  ///
  /// Searches for vertex pairs within a distance threshold of 0.01 units.
  /// Useful for detecting mesh issues or merging vertices.
  void findSimilarVertexes() {
    std::vector<std::pair<int, int>> similarities;

    for (int i = 0; i < points.size(); ++i)
      for (int j = 0; j < points.size(); ++j) {
        if (i == j) continue;
        FT a = (points[i][0] - points[j][0]), b = (points[i][1] - points[j][1]), c = (points[i][2] - points[j][2]);
        FT dist = std::sqrt(a * a + b * b + c * c);
        if (dist < FT(0.01)) similarities.push_back(std::pair<int, int>(i, j));
      }

    // for(int i = 0; i < similarities.size(); ++i)
    //     std::cout << "similar: " << similarities[i].first << "  " << similarities[i].second << std::endl;
  }
};

/// @brief Stream output operator for Object3D.
/// @tparam FT Floating-point type.
/// @param os Output stream.
/// @param obj Object to write.
/// @return Reference to output stream.
template <class FT>
std::ostream& operator<<(std::ostream& os, const Object3D<FT>& obj) {
  obj.write(os);
  return os;
}

/// @brief Stream input operator for Object3D.
/// @tparam FT Floating-point type.
/// @param is Input stream.
/// @param obj Object to read into.
/// @return Reference to input stream.
template <class FT>
std::istream& operator<<(std::istream& is, Object3D<FT>& obj) {
  obj.read(is);
  return is;
}

/// @brief Type alias for list of 3D objects.
template <class FT>
using Object3DList = std::vector<Object3D<FT>>;

/// @brief Load multiple objects from OBJ stream.
///
/// Reads all objects from an OBJ file stream, handling multi-object files
/// with proper vertex index offsets.
/// @tparam FT Floating-point type.
/// @param is Input stream.
/// @param data Output object list.
template <class FT>
void loadObjects(std::istream& is, Object3DList<FT>& data) {
  Object3D<FT> obj;
  int pointCount =
      0;  // need to keep offset to identify points of each object, otherwise their index is incremented continously
  int normalCount = 0;  // same as above
  int texCount = 0;     // same as above
  while (is.good()) {
    obj.read(is, pointCount, normalCount, texCount);
    if (obj.points.size()) data.push_back(obj);
    obj.findSimilarVertexes();
  }
}
/*
    template<class FT>
    bool loadObjects(std::string &file, Object3DList<FT> &data) {
        return loadObjects(file,data);
    }
*/

/// @brief Load objects from OBJ file.
/// @tparam FT Floating-point type.
/// @param file Path to OBJ file.
/// @param data Output object list.
/// @return True if file opened successfully.
template <class FT>
bool loadObjects(const std::string& file, Object3DList<FT>& data) {
  std::fstream fs;
  fs.open(file, std::ios_base::in);
  if (fs.is_open()) {
    loadObjects(fs, data);
    return true;
  }
  return false;
}

/// @brief Save objects to OBJ stream.
/// @tparam FT Floating-point type.
/// @param os Output stream.
/// @param data Objects to save.
template <class FT>
void saveObjects(std::ostream& os, const Object3DList<FT>& data) {
  os << "# obj file generator" << std::endl;
  for_each(data.begin(), data.end(), [&](const Object3D<FT>& obj) { obj.write(os); });
}

/// @brief Save objects to OBJ file (non-const path).
/// @tparam FT Floating-point type.
/// @param file Path to OBJ file.
/// @param data Objects to save.
/// @return True if file opened successfully.
template <class FT>
bool saveObjects(std::string& file, Object3DList<FT>& data) {
  return saveObjects(file, data);
}

/// @brief Save objects to OBJ file.
/// @tparam FT Floating-point type.
/// @param file Path to OBJ file.
/// @param data Objects to save.
/// @return True if file opened successfully.
template <class FT>
bool saveObjects(const std::string& file, Object3DList<FT>& data) {
  std::fstream fs;
  fs.open(file, std::ios_base::out | std::ios_base::trunc);
  if (fs.is_open()) {
    saveObjects(fs, data);
    return true;
  }
  return false;
}

}  // namespace lsfm
