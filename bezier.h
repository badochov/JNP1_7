#ifndef JNP1_7_BEZIER_H
#define JNP1_7_BEZIER_H

#include <cmath>
#include <functional>
#include <iostream>

namespace bezier {
    namespace types {
        using real_t = long double;
        using node_index_t = size_t;

        class point_2d {
        public:
            const real_t X;
            const real_t Y;

            constexpr point_2d(real_t x, real_t y) : X(x), Y(y) {
            }

            bool operator==(const point_2d &other) const {
                return other.X == X && other.Y == Y;
            }

            point_2d operator*(real_t scalar) const {
                return point_2d(X * scalar, Y * scalar);
            }

            friend point_2d operator*(real_t scalar, const point_2d &point) {
                return point_2d(point.X * scalar, point.Y * scalar);
            }

            point_2d operator+(const point_2d &point) const {
                return point_2d(X + point.X, Y + point.Y);
            }

            friend std::ostream &operator<<(std::ostream &os, const point_2d &point) {
                os << "(" << point.X << ", " << point.Y << ")";
                return os;
            }
        };

        using point_function_t = std::function<const point_2d(node_index_t)>;
    } // namespace types

    namespace constants {
        constexpr types::node_index_t NUM_OF_CUBIC_BEZIER_NODES = 4;
    }

    namespace detail {
        const types::real_t ARC = 4 * (std::sqrt(2) - 1) / 3;
        // Degrees to radians converter;
        const types::real_t DEG_TO_RAD = M_PI / 180;

        // Helper points for predefined functions returning points.
        constexpr types::point_2d neg_pos(-1, 1);
        constexpr types::point_2d neg_neg(-1, -1);
        constexpr types::point_2d pos_neg(1, -1);
        constexpr types::point_2d pos_pos(1, 1);
        constexpr types::point_2d zero_pos(0, 1);
        constexpr types::point_2d pos_zero(1, 0);

        // Helper function to return function that return i-th point
        // or throws std::out_of_range if i is larger than constants::NUM_OF_CUBIC_BEZIER_NODES.
        types::point_function_t four_points_function(const types::point_2d &point0,
                                                     const types::point_2d &point1,
                                                     const types::point_2d &point2,
                                                     const types::point_2d &point3) {
            return [=](types::node_index_t i) {
                return i == 3 ? point3
                              : i == 2 ? point2
                                       : i == 1 ? point1
                                                : i == 0 ? point0
                                                         : (throw std::out_of_range(
                                                        "a curve node index is out of range"));
            };
        }

    } // namespace detail

    types::point_function_t Cup() {
        static const types::point_function_t cup = detail::four_points_function(
                detail::neg_pos, detail::neg_neg, detail::pos_neg, detail::pos_pos);
        return cup;
    }

    types::point_function_t Cap() {
        static const types::point_function_t cap = detail::four_points_function(
                detail::neg_neg, detail::neg_pos, detail::pos_pos, detail::pos_neg);
        return cap;
    }

    types::point_function_t ConvexArc() {
        static const types::point_function_t convex_arc =
                detail::four_points_function(detail::zero_pos, types::point_2d(detail::ARC, 1),
                                             types::point_2d(1, detail::ARC), detail::pos_zero);
        return convex_arc;
    }

    types::point_function_t ConcaveArc() {
        static const types::point_function_t concave_arc =
                detail::four_points_function(detail::zero_pos, types::point_2d(0, 1 - detail::ARC),
                                             types::point_2d(1 - detail::ARC, 0), detail::pos_zero);
        return concave_arc;
    }

    types::point_function_t LineSegment(const types::point_2d &p, const types::point_2d &q) {
        return detail::four_points_function(p, p, q, q);
    }

    types::point_function_t MovePoint(types::point_function_t f, types::node_index_t i,
                                      types::real_t x, types::real_t y) {
        return [f = std::move(f), i, x, y](types::node_index_t idx) {
            const types::point_2d v(x, y);
            types::point_2d point = f(idx);
            return idx == i ? point + v : point;
        };
    }

    types::point_function_t Rotate(types::point_function_t f, types::real_t a) {
        return [f = std::move(f), a](types::node_index_t idx) {
            const types::real_t rad_a = a * detail::DEG_TO_RAD;
            const types::real_t _sin = std::sin(rad_a);
            const types::real_t _cos = std::cos(rad_a);

            types::point_2d point = f(idx);

            types::real_t x = point.X * _cos - point.Y * _sin;
            types::real_t y = point.X * _sin + point.Y * _cos;
            return types::point_2d(x, y);
        };
    }

    types::point_function_t Scale(types::point_function_t f, types::real_t x, types::real_t y) {
        return [f = std::move(f), x, y](types::node_index_t idx) {
            types::point_2d point = f(idx);
            return types::point_2d(point.X * x, point.Y * y);
        };
    }

    types::point_function_t Translate(types::point_function_t f, types::real_t x, types::real_t y) {
        return [f = std::move(f), x, y](types::node_index_t idx) {
            const types::point_2d v(x, y);

            types::point_2d point = f(idx);
            return point + v;
        };
    }

    types::point_function_t Concatenate(types::point_function_t f1, types::point_function_t f2) {
        return [f1 = std::move(f1), f2 = std::move(f2)](types::node_index_t i) {
            return i < constants::NUM_OF_CUBIC_BEZIER_NODES
                   ? f1(i)
                   : f2(i - constants::NUM_OF_CUBIC_BEZIER_NODES);
        };
    }

    template<typename... Functions>
    types::point_function_t Concatenate(types::point_function_t f1, types::point_function_t f2,
                                        Functions... fs) {
        return Concatenate(f1, Concatenate(f2, fs...));
    }

    class P3CurvePlotter {
        using should_be_printed_fn_t = std::function<bool(size_t, size_t)>;

        static constexpr types::real_t MIN_RANGE = 0;
        static constexpr types::real_t MAX_RANGE = 1;
        static constexpr types::real_t MIN_Y = -1;
        static constexpr types::real_t MAX_Y = 1;
        static constexpr types::real_t MIN_X = -1;
        static constexpr types::real_t MAX_X = 1;

        should_be_printed_fn_t should_be_printed = [](size_t, size_t) { return false; };
        size_t resolution;

        // Returns nth segment of equally divided into 'size' segments [MIN_RANGE, MAX_RANGE].
        [[nodiscard]] static types::real_t get_coord(size_t n, size_t size) {
            return MIN_RANGE + (MAX_RANGE - MIN_RANGE) / size * n;
        }

        [[nodiscard]] static bool in_area(const types::point_2d &point) {
            return point.X <= MAX_X && point.X >= MIN_X && point.Y >= MIN_Y && point.Y <= MAX_Y;
        }

        void add_to_printed(const types::point_2d &point) {
            static const types::point_2d normalizer(-MIN_X, -MIN_Y);
            static const types::real_t y_normalization = ((types::real_t) (resolution - 1) / (MAX_Y - MIN_Y));
            static const types::real_t x_normalization = ((types::real_t) (resolution - 1) / (MAX_X - MIN_X));

            types::point_2d normalized = point + normalizer;

            types::real_t x =
                    std::round(normalized.X * x_normalization);
            types::real_t y =
                    std::round(normalized.Y * y_normalization);

            should_be_printed = [prev = std::move(should_be_printed), x, y](size_t _x, size_t _y) {
                return (x == _x && y == _y) || prev(_x, _y);
            };
        }

        void discard(const types::point_2d &point) const {};

        void end() const {};

        void print_column(size_t column, size_t row, std::ostream &os, char fg, char bg) const {
            os << (should_be_printed(column, row) ? fg : bg);
            column == resolution - 1 ? end() : print_column(column + 1, row, os, fg, bg);
        }

        void print_row(size_t row, std::ostream &os, char fg, char bg) const {
            print_column(0, row, os, fg, bg);
            os << std::endl;
            row == 0 ? end() : print_row(row - 1, os, fg, bg);
        }

        // Evaluates Bezier curve and adds generated point to printed if is in range.
        // size - amount of points that should be generated.
        // i - which of those 'size' points is evaluated.
        void add_segment_point(const std::function<types::point_2d(types::real_t)> &f, size_t i,
                               size_t size) {
            types::real_t t = get_coord(i, size);
            types::point_2d point = f(t);
            in_area(point) ? add_to_printed(point) : discard(point);

            i == size - 1 ? end() : add_segment_point(f, i + 1, size);
        }

        // Processes segment of point returning function.
        // Recursively calls itself until all segments are processed.
        void process_segment(const types::point_function_t &f, size_t segment, size_t segments,
                             size_t size) {
            size_t segment_size = std::ceil((types::real_t) size / segments);

            add_segment_point([&](types::real_t t) { return (*this)(f, t, segment); }, 0,
                              segment_size);

            segments == 1 ? end()
                          : process_segment(f, segment + 1, segments - 1, size - segment_size);
        }

    public:
        explicit P3CurvePlotter(const types::point_function_t &f, size_t segments = 1,
                                size_t _resolution = 80)
                : resolution(_resolution) {
            size_t size = resolution * resolution;
            process_segment(f, 0, segments, size);
        }

        void Print(std::ostream &os = std::cout, char fg = '*', char bg = ' ') const {
            print_row(resolution, os, fg, bg);
        }

        types::point_2d operator()(const types::point_function_t &f, types::real_t t,
                                   types::node_index_t segment) const {
            const types::node_index_t index_change = segment * constants::NUM_OF_CUBIC_BEZIER_NODES;

            types::point_2d p0 = f(index_change + 0);
            types::point_2d p1 = f(index_change + 1);
            types::point_2d p2 = f(index_change + 2);
            types::point_2d p3 = f(index_change + 3);

            types::point_2d b0 = (1 - t) * p0 + t * p1;
            types::point_2d b1 = (1 - t) * p1 + t * p2;
            types::point_2d b2 = (1 - t) * p2 + t * p3;

            types::point_2d c0 = (1 - t) * b0 + t * b1;
            types::point_2d c1 = (1 - t) * b1 + t * b2;

            return (1 - t) * c0 + t * c1;
        }
    };

} // namespace bezier

#endif // JNP1_7_BEZIER_H
