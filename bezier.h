#ifndef JNP1_7_BEZIER_H
#define JNP1_7_BEZIER_H

#include <cmath>
#include <memory>
#include <functional>
#include <iostream>
#include <utility>

namespace bezier {
    namespace types {
        using real_t = long double;
        using node_index_t = size_t;
        using segment_number_t = size_t;

        // Functional list class. With head and tail.
        template<typename T>
        class List {
        public:
            using list_t = std::shared_ptr<List<T>>;

            const T head;
            const list_t tail;
        private:
            List(const T _el, const list_t _tail) : head(_el), tail(std::move(_tail)) {
            }

            // Helper function for unique function.
            static list_t unique_helper(const list_t &_list, const T &prev_el, const list_t &acc) {
                return is_empty(_list) ?
                       rev(acc) :
                       (_list->head == prev_el ?
                        unique_helper(_list->tail, prev_el, acc) :
                        unique_helper(_list->tail, _list->head, make_list(_list->head, acc)));
            }

            // Helper function for concat function.
            static list_t concat_helper(const list_t &_list2, const list_t &acc) {
                return is_empty(_list2) ? acc : concat_helper(_list2->tail, make_list(_list2->head, acc));
            }

            // Helper function for spli function.
            static std::pair<list_t, list_t> split_helper(const list_t &src, const list_t &left, const list_t &right) {
                return is_empty(src) ? std::make_pair(left, right) : split_helper(src->tail, right,
                                                                                  make_list(src->head, left));
            }

            // Helper function for merge sort. Splits list into two lists with ~sane amount of elements.
            static std::pair<list_t, list_t> split(const list_t &_list) {
                return split_helper(_list, get_empty(), get_empty());
            }

            // Helper function for merge function.
            static list_t merge_helper(const list_t &_list, const list_t &_list2, const list_t &acc) {
                return (is_empty(_list) && is_empty(_list2)) ?
                       acc : (
                               is_empty(_list) ?
                               concat_helper(_list2, acc) : (
                                       is_empty(_list2) ?
                                       concat_helper(_list, acc) : (
                                               _list->head < _list2->head ?
                                               merge_helper(_list->tail, _list2, make_list(_list->head, acc)) :
                                               merge_helper(_list, _list2->tail, make_list(_list2->head, acc))
                                       )
                               )
                       );
            }

            // Helper function for reverse function.
            static list_t rev_helper(const list_t &_list, const list_t &acc) {
                return is_empty(_list) ? acc : rev_helper(_list->tail, make_list(_list->head, acc));
            }

        public:
            // Creates pointer to list.
            static list_t make_list(const T _el, list_t _tail) {
                return std::shared_ptr<List<T>>(new List<T>(_el, _tail));
            }

            // Returns empty list.
            static list_t get_empty() {
                return nullptr;
            }

            // Checks if list is empty.
            static bool is_empty(const list_t &_list) {
                return _list == nullptr;
            }

            // Reverts list.
            static list_t rev(const list_t &_list) {
                return rev_helper(_list, get_empty());
            }

            // Merges two sorted lists into one sorted list.
            static list_t merge(const list_t &_list, const list_t &_list2) {
                return rev(merge_helper(_list, _list2, get_empty()));
            }

            // Concatenates two lists.
            static list_t concat(const list_t &_list, const list_t &_list2) {
                return rev(concat_helper(_list2, rev(_list)));
            }

            // Sorts list.
            static list_t sort(const list_t &_list) {
                if (is_empty(_list) || is_empty(_list->tail)) {
                    return _list;
                }

                auto[l1, l2] = split(_list);
                return merge(sort(l1), sort(l2));
            }

            // Removes duplicates from sorted list.
            static list_t unique(const list_t &_list) {
                return is_empty(_list) ? _list : make_list(_list->head,
                                                           unique_helper(_list->tail, _list->head, get_empty()));
            }

        };

        class point_2d {
        public:
            const real_t X;
            const real_t Y;

            constexpr point_2d(const real_t x, const real_t y) : X(x), Y(y) {
            }

            bool operator==(const point_2d &other) const {
                return other.X == X && other.Y == Y;
            }

            point_2d operator*(const real_t scalar) const {
                return point_2d(X * scalar, Y * scalar);
            }

            friend point_2d operator*(const real_t scalar, const point_2d &point) {
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

        using point_function_t = std::function<const point_2d(const node_index_t)>;
    } // namespace types

    namespace constants {
        constexpr types::node_index_t NUM_OF_CUBIC_BEZIER_NODES = 4;

        constexpr size_t DEFAULT_RESOLUTION = 80;
        constexpr types::segment_number_t DEFAULT_NUM_SEGMENTS = 1;

        constexpr char DEFAULT_CURVE_CHAR = '*';
        constexpr char DEFAULT_BACKGROUND_CHAR = ' ';

        const types::real_t ARC = 4. * (std::sqrt(2) - 1.) / 3.;

        // Degrees to radians converter;
        constexpr types::real_t DEG_TO_RAD = M_PI / 180.;

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
            return [=](const types::node_index_t i) {
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

    inline types::point_function_t MovePoint(const types::point_function_t &f, const types::node_index_t i,
                                             const types::real_t x, const types::real_t y) {
        return [=](const types::node_index_t idx) {
            const types::point_2d v(x, y);
            const types::point_2d point = f(idx);

            return idx == i ? point + v : point;
        };
    }

    // Rotates function by given in degrees angle.
    inline types::point_function_t Rotate(const types::point_function_t &f, const types::real_t a) {
        return [=](const types::node_index_t idx) {
            const types::real_t rad_a = a * constants::DEG_TO_RAD;
            const types::real_t _sin = std::sin(rad_a);
            const types::real_t _cos = std::cos(rad_a);

            const types::point_2d point = f(idx);

            const types::real_t x = point.X * _cos - point.Y * _sin;
            const types::real_t y = point.X * _sin + point.Y * _cos;

            return types::point_2d(x, y);
        };
    }

    // Scales point returned by function by multiplying resulting point with given coefficients.
    inline types::point_function_t
    Scale(const types::point_function_t &f, const types::real_t x, const types::real_t y) {
        return [=](const types::node_index_t idx) {
            const types::point_2d point = f(idx);
            return types::point_2d(point.X * x, point.Y * y);
        };
    }

    // Moves point returned by function by given vector.
    inline types::point_function_t
    Translate(const types::point_function_t &f, const types::real_t x, const types::real_t y) {
        return [=](const types::node_index_t idx) {
            const types::point_2d v(x, y);

            const types::point_2d point = f(idx);
            return v + point;
        };
    }

    // Concatenates f2 with first segment of f1.
    inline types::point_function_t Concatenate(const types::point_function_t &f1, const types::point_function_t &f2) {
        return [=](const types::node_index_t i) {
            return i < constants::NUM_OF_CUBIC_BEZIER_NODES
                   ? f1(i)
                   : f2(i - constants::NUM_OF_CUBIC_BEZIER_NODES);
        };
    }

    // Generic version of Concatenate function.
    template<typename... Functions>
    inline types::point_function_t Concatenate(const types::point_function_t &f1, const types::point_function_t &f2,
                                               Functions &&... fs) {
        return Concatenate(f1, Concatenate(f2, fs...));
    }

    // Class for plotting curve.
    class P3CurvePlotter {
        using point = std::pair<size_t, size_t>;

        // Range of variables that can be passed to function.
        static constexpr types::real_t MIN_RANGE = 0;
        static constexpr types::real_t MAX_RANGE = 1;

        // Cords of part of plane that will be plotted.
        static constexpr types::real_t MIN_Y = -1;
        static constexpr types::real_t MAX_Y = 1;
        static constexpr types::real_t MIN_X = -1;
        static constexpr types::real_t MAX_X = 1;

        using points_to_print_base_t = types::List<point>;
        using points_to_print_t = types::List<point>::list_t;

        // Image resolution.
        const size_t resolution;
        // Point that should be printed in order from left top to right bottom.
        const points_to_print_t points_to_print;


        // Returns nth segment of equally divided into 'size' segments [MIN_RANGE, MAX_RANGE].
        [[nodiscard]] static types::real_t get_coord(const size_t n, const size_t size) {
            return MIN_RANGE + (MAX_RANGE - MIN_RANGE) / size * n;
        }

        // Checks if given point is in part of the plane that is plotted.
        [[nodiscard]] static bool in_area(const types::point_2d &point) {
            return point.X <= MAX_X && point.X >= MIN_X && point.Y >= MIN_Y && point.Y <= MAX_Y;
        }

        // Maps point on plane to output point.
        [[nodiscard]] point create_point(const types::point_2d &point) const {
            static const types::point_2d normalizer(-MIN_X, -MIN_Y);

            const types::real_t y_normalization = ((types::real_t) (resolution - 1) / (MAX_Y - MIN_Y));
            const types::real_t x_normalization = ((types::real_t) (resolution - 1) / (MAX_X - MIN_X));

            const types::point_2d normalized = point + normalizer;

            const types::real_t first_coord = resolution - std::round(normalized.Y * y_normalization);
            const types::real_t second_coord = std::round(normalized.X * x_normalization);

            return {first_coord, second_coord};
        }

        // Evaluates Bezier curve and adds generated point to printed if is in range.
        [[nodiscard]] points_to_print_t
        add_segment_point(const std::function<types::point_2d(types::real_t)> &f, const size_t i,
                          const size_t size, const points_to_print_t &should) const {
            const types::real_t t = get_coord(i, size);
            const types::point_2d point = f(t);
            const P3CurvePlotter::point p = create_point(point);

            const points_to_print_t print = in_area(point) ?
                                            points_to_print_base_t::make_list(p, should)
                                                           : should;

            return i == size - 1 ? print : add_segment_point(f, i + 1, size, print);
        }

        // Processes segment of point returning function.
        // Recursively calls itself until all segments are processed.
        [[nodiscard]] points_to_print_t
        process_segment(const types::point_function_t &f, const types::segment_number_t segment,
                        const types::segment_number_t segments,
                        const size_t size, const points_to_print_t &should) const {
            const size_t segment_size = std::ceil((types::real_t) size / segments);

            const points_to_print_t print = add_segment_point(
                    std::bind(&P3CurvePlotter::operator(), this, std::cref(f), std::placeholders::_1, segment),
                    0,
                    segment_size,
                    should);

            return segments == 1 ?
                   print :
                   process_segment(f, segment + 1, segments - 1, size - segment_size, print);
        }

        [[nodiscard]] points_to_print_t
        get_points_to_print(const types::point_function_t &f, const types::segment_number_t segments) const {
            const points_to_print_t points = process_segment(
                    f, 0, segments, resolution * resolution, points_to_print_base_t::get_empty());
            const points_to_print_t sorted = points_to_print_base_t::sort(points);
            return points_to_print_base_t::unique(sorted);
        }

        static bool
        print(const points_to_print_t &should, const size_t column, const size_t row,
              std::ostream &os, char fg, char bg) {
            return (!points_to_print_base_t::is_empty(should) && should->head == point(row, column)) ?
                   (os << fg && true) :
                   (os << bg && false);
        }

        points_to_print_t
        print_column(const points_to_print_t &should, const size_t column, const size_t row,
                     std::ostream &os, char fg, char bg) const {
            return column == resolution ?
                   should :
                   print_column(
                           (print(should, column, row, os, fg, bg) ? should->tail : should),
                           column + 1, row, os, fg, bg);
        }

        points_to_print_t
        print_row(const points_to_print_t &should, const size_t row, std::ostream &os, char fg, char bg) const {
            if (row == resolution) {
                return should;
            }
            const points_to_print_t print = print_column(should, 0, row, os, fg, bg);
            return print_row(print, row + 1, os << std::endl, fg, bg);
        }

    public:
        explicit P3CurvePlotter(const types::point_function_t &f,
                                const types::segment_number_t segments = constants::DEFAULT_NUM_SEGMENTS,
                                const size_t _resolution = constants::DEFAULT_RESOLUTION)
                : resolution(_resolution),
                  points_to_print(get_points_to_print(f, segments)) {
        }


        void Print(std::ostream &os = std::cout,
                   const char fg = constants::DEFAULT_CURVE_CHAR,
                   const char bg = constants::DEFAULT_BACKGROUND_CHAR) const {
            print_row(points_to_print, 0, os, fg, bg);
        }

        // Calculates B(t) with param points of B obtained from given segment of passed function.
        types::point_2d
        operator()(const types::point_function_t &f, const types::real_t t,
                   const types::segment_number_t segment) const {
            const types::node_index_t index_change = segment * constants::NUM_OF_CUBIC_BEZIER_NODES;

            const types::point_2d p0 = f(index_change + 0);
            const types::point_2d p1 = f(index_change + 1);
            const types::point_2d p2 = f(index_change + 2);
            const types::point_2d p3 = f(index_change + 3);


            return p0 * std::pow((1 - t), 3) +
                   3 * p1 * t * std::pow((1 - t), 2) +
                   3 * p2 * std::pow(t, 2) * (1 - t) +
                   p3 * std::pow(t, 3);
        }
    };

} // namespace bezier

#endif // JNP1_7_BEZIER_H
