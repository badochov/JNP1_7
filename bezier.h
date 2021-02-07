#ifndef JNP1_7_BEZIER_H
#define JNP1_7_BEZIER_H

#include <cmath>
#include <memory>
#include <functional>
#include <optional>
#include <iostream>
#include <utility>

namespace bezier {
    namespace types {
        using real_t = long double;
        using node_index_t = size_t;


        template<typename T>
        struct List {
            const T el;
            using list_t = std::shared_ptr<List<T>>;
            const list_t tail;

            static list_t get_empty() {
                return nullptr;
            }

            static bool is_empty(const list_t &list) {
                return list == nullptr;
            }

            static T get(const list_t &list, size_t n) {
                return n == 0 ? list->el : get(list->tail, n - 1);
            }


            static list_t set(const list_t &list, size_t n, const T &el) {
                return n == 0 ?
                       std::make_shared<List<T>>(el, list->tail) :
                       std::make_shared<List<T>>(std::move(list->el), set(std::move(list->tail), n - 1, el));
            }

            static std::ostream &print(const list_t &list, std::ostream &os, char fg, char bg) {
                return is_empty(list) ? os : print(list->tail, os << (list->el ? fg : bg), fg, bg);
            }

            List(const T _el, list_t _tail) : el(_el), tail(std::move(_tail)) {

            }

            List(size_t size, const T _el) : el(_el),
                                             tail(size == 0 ? get_empty() : std::make_shared<List<T>>(size - 1, _el)) {

            }

        };

        template<typename T>
        struct List2D {

            using list_t = std::shared_ptr<List<T>>;
            using list2d_t = std::shared_ptr<List2D<T>>;
            const list_t el;
            const list2d_t tail;

            static list2d_t get_empty() {
                return nullptr;
            }

            static bool is_empty(const list2d_t list) {
                return list == nullptr;
            }

            static T get(const list2d_t list, size_t row, size_t col) {
                return row == 0 ? List<T>::get(list->el, col) : get(list->tail, row - 1, col);
            }

            static list2d_t set(const list2d_t list, size_t row, size_t col, T el) {
                return row == 0 ?
                       std::make_shared<List2D<T>>(List<T>::set(list->el, col, el), list->tail)
                                : std::make_shared<List2D<T>>(list->el, set(list->tail, row - 1, col, el));
            }

            static size_t size(const list2d_t list) {
                return is_empty(list) ? 0 : 1 + size(list->tail);
            }

            static std::ostream &print(const list2d_t list, std::ostream &os, char fg, char bg) {
                return is_empty(list) ?
                       os
                                      : print(list->tail, List<T>::print(list->el, os, fg, bg) << std::endl, fg, bg);
            }


            List2D(const list_t _el, const list2d_t _tail) : el(_el), tail(_tail) {

            }

            List2D(size_t rows, size_t cols, const T _el) : el(std::make_shared<List<T>>(cols, _el)),
                                                            tail(rows == 0 ?
                                                                 get_empty()
                                                                           : std::make_shared<List2D<T>>(rows - 1, cols,
                                                                                                         _el)) {

            }
        };


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


        const types::real_t ARC = 4. * (std::sqrt(2) - 1.) / 3.;
        // Degrees to radians converter;
        const types::real_t DEG_TO_RAD = M_PI / 180.;

        // Helper points for predefined functions returning points.
        constexpr types::point_2d neg_pos(-1., 1.);
        constexpr types::point_2d neg_neg(-1., -1.);
        constexpr types::point_2d pos_neg(1., -1.);
        constexpr types::point_2d pos_pos(1., 1.);
        constexpr types::point_2d zero_pos(0., 1.);
        constexpr types::point_2d pos_zero(1., 0.);
    }

    namespace detail {

        // Helper function to return function that return i-th point
        // or throws std::out_of_range if i is larger than constants::NUM_OF_CUBIC_BEZIER_NODES.
        inline types::point_function_t four_points_function(const types::point_2d &point0,
                                                            const types::point_2d &point1,
                                                            const types::point_2d &point2,
                                                            const types::point_2d &point3) {
            return [point0, point1, point2, point3](types::node_index_t i) {
                return i == 3 ? point3 :
                       i == 2 ? point2 :
                       i == 1 ? point1 :
                       i == 0 ? point0 :
                       (throw std::out_of_range("a curve node index is out of range"));
            };
        }

    } // namespace detail

    inline types::point_function_t Cup() {
        return detail::four_points_function(
                constants::neg_pos, constants::neg_neg, constants::pos_neg, constants::pos_pos);
    }

    inline types::point_function_t Cap() {
        return detail::four_points_function(
                constants::neg_neg, constants::neg_pos, constants::pos_pos, constants::pos_neg);
    }

    inline types::point_function_t ConvexArc() {
        return detail::four_points_function(constants::zero_pos, types::point_2d(constants::ARC, 1.),
                                            types::point_2d(1., constants::ARC), constants::pos_zero);
    }

    inline types::point_function_t ConcaveArc() {
        return detail::four_points_function(constants::zero_pos, types::point_2d(0., 1. - constants::ARC),
                                            types::point_2d(1. - constants::ARC, 0.), constants::pos_zero);
    }

    inline types::point_function_t LineSegment(const types::point_2d &p, const types::point_2d &q) {
        return detail::four_points_function(p, p, q, q);
    }

    inline types::point_function_t MovePoint(const types::point_function_t &f, types::node_index_t i,
                                             types::real_t x, types::real_t y) {
        return [f, i, x, y](types::node_index_t idx) {
            const types::point_2d v(x, y);
            const types::point_2d point = f(idx);
            return idx == i ? point + v : point;
        };
    }

    inline types::point_function_t Rotate(const types::point_function_t &f, types::real_t a) {
        return [f, a](types::node_index_t idx) {
            const types::real_t rad_a = a * constants::DEG_TO_RAD;
            const types::real_t _sin = std::sin(rad_a);
            const types::real_t _cos = std::cos(rad_a);

            const types::point_2d point = f(idx);

            const types::real_t x = point.X * _cos - point.Y * _sin;
            const types::real_t y = point.X * _sin + point.Y * _cos;

            return types::point_2d(x, y);
        };
    }

    inline types::point_function_t Scale(const types::point_function_t &f, types::real_t x, types::real_t y) {
        return [f, x, y](types::node_index_t idx) {
            const types::point_2d point = f(idx);
            return types::point_2d(point.X * x, point.Y * y);
        };
    }

    inline types::point_function_t Translate(const types::point_function_t &f, types::real_t x, types::real_t y) {
        return [f, x, y](types::node_index_t idx) {
            const types::point_2d v(x, y);

            const types::point_2d point = f(idx);
            return v + point;
        };
    }

    inline types::point_function_t Concatenate(const types::point_function_t &f1, const types::point_function_t &f2) {
        return [f1, f2](types::node_index_t i) {
            return i < constants::NUM_OF_CUBIC_BEZIER_NODES
                   ? f1(i)
                   : f2(i - constants::NUM_OF_CUBIC_BEZIER_NODES);
        };
    }

    template<typename... Functions>
    inline types::point_function_t Concatenate(const types::point_function_t &f1, const types::point_function_t &f2,
                                               Functions &&... fs) {
        return Concatenate(f1, Concatenate(f2, fs...));
    }

    inline const types::point_2d calc(const types::point_function_t &f, types::real_t t,
                                      types::node_index_t segment) {
        const types::node_index_t index_change = segment * constants::NUM_OF_CUBIC_BEZIER_NODES;

        const types::point_2d p0 = f(index_change + 0);
        const types::point_2d p1 = f(index_change + 1);
        const types::point_2d p2 = f(index_change + 2);
        const types::point_2d p3 = f(index_change + 3);

        const types::point_2d b0 = (1 - t) * p0 + t * p1;
        const types::point_2d b1 = (1 - t) * p1 + t * p2;
        const types::point_2d b2 = (1 - t) * p2 + t * p3;

        const types::point_2d c0 = (1 - t) * b0 + t * b1;
        const types::point_2d c1 = (1 - t) * b1 + t * b2;

        return (1 - t) * c0 + t * c1;
    }

    inline std::function<types::point_2d()> c() {
        return []() { return constants::pos_pos; };
    }

    class P3CurvePlotter {
        using point = std::pair<size_t, size_t>;

        static constexpr types::real_t MIN_RANGE = 0;
        static constexpr types::real_t MAX_RANGE = 1;
        static constexpr types::real_t MIN_Y = -1;
        static constexpr types::real_t MAX_Y = 1;
        static constexpr types::real_t MIN_X = -1;
        static constexpr types::real_t MAX_X = 1;

        using should_be_printed_t = types::List2D<bool>;
        using should_be_printed_ptr_t = types::List2D<bool>::list2d_t;

        const size_t resolution;
        const should_be_printed_ptr_t should_be_printed;


        // Returns nth segment of equally divided into 'size' segments [MIN_RANGE, MAX_RANGE].
        [[nodiscard]] static types::real_t get_coord(size_t n, size_t size) {
            return MIN_RANGE + (MAX_RANGE - MIN_RANGE) / size * n;
        }

        [[nodiscard]] static bool in_area(const types::point_2d &point) {
            return point.X <= MAX_X && point.X >= MIN_X && point.Y >= MIN_Y && point.Y <= MAX_Y;
        }

        [[nodiscard]] point create_point(const types::point_2d &point) const {
            static const types::point_2d normalizer(-MIN_X, -MIN_Y);

            const types::real_t y_normalization = ((types::real_t) (resolution - 1) / (MAX_Y - MIN_Y));
            const types::real_t x_normalization = ((types::real_t) (resolution - 1) / (MAX_X - MIN_X));

            const types::point_2d normalized = point + normalizer;

            const types::real_t x = std::round(normalized.X * x_normalization);
            const types::real_t y = resolution - std::round(normalized.Y * y_normalization);

            return {y, x};
        }

        // Evaluates Bezier curve and adds generated point to printed if is in range.
        // size - amount of points that should be generated.
        // i - which of those 'size' points is evaluated.
        should_be_printed_ptr_t
        add_segment_point(const std::function<types::point_2d(types::real_t)> &f, size_t i,
                          size_t size, const should_be_printed_ptr_t &should) {
            const types::real_t t = get_coord(i, size);
//            const types::point_2d point = Cup()(0);
            const types::point_2d point = f(t);
            const P3CurvePlotter::point p = create_point(point);

//            return i == size - 1 ? should : add_segment_point(f, i + 1, size, should);
            should_be_printed_ptr_t print = in_area(point) ?
                                            should_be_printed_t::set(should, p.first, p.second, true)
                                                           : should;

            return i == size - 1 ? print : add_segment_point(f, i + 1, size, print);
        }

        // Processes segment of point returning function.
        // Recursively calls itself until all segments are processed.
        should_be_printed_ptr_t process_segment(const types::point_function_t &f, size_t segment, size_t segments,
                                                size_t size, const should_be_printed_ptr_t &should) {
            const size_t segment_size = std::ceil((types::real_t) size / segments);

            const should_be_printed_ptr_t print = add_segment_point(
                    std::bind(calc, std::cref(f), std::placeholders::_1, segment),
//                    [f, segment](types::real_t t) { return calc(f, t, segment); },
                    0,
                    segment_size,
                    should);

            return segments == 1 ?
                   print :
                   process_segment(f, segment + 1, segments - 1, size - segment_size, print);
        }

    public:
        explicit P3CurvePlotter(const types::point_function_t &f, size_t segments = 1,
                                size_t _resolution = 80)
                : resolution(_resolution), should_be_printed(process_segment(f, 0, segments, _resolution * _resolution,
                                                                             std::make_shared<should_be_printed_t>(
                                                                                     _resolution, _resolution,
                                                                                     false))) {
        }


        void Print(std::ostream &os = std::cout, char fg = '*', char bg = ' ') const {
//            os << should_be_printed_t::get(should_be_printed, resolution - 1, resolution - 1);;
//            return;
            types::List2D<bool>::print(should_be_printed, os, fg, bg);
        }

        types::point_2d operator()(const types::point_function_t &f, types::real_t t,
                                   types::node_index_t segment) const {
            return calc(f, t, segment);
        }
    };

} // namespace bezier

#endif // JNP1_7_BEZIER_H
