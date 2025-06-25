#ifndef MATCH_PARAMETERS_GRAMMAR_HPP
#define MATCH_PARAMETERS_GRAMMAR_HPP

#include "server/api/route_parameters_grammar.hpp"
#include "engine/api/match_parameters.hpp"
#include "engine/yaw_rate.hpp"
#include "engine/steering_angle.hpp"

#include <boost/phoenix.hpp>
#include <boost/spirit/include/qi.hpp>

namespace osrm::server::api
{

namespace
{
namespace ph = boost::phoenix;
namespace qi = boost::spirit::qi;
} // namespace

template <typename Iterator = std::string::iterator,
          typename Signature = void(engine::api::MatchParameters &)>
struct MatchParametersGrammar final : public RouteParametersGrammar<Iterator, Signature>
{
    using BaseGrammar = RouteParametersGrammar<Iterator, Signature>;

    MatchParametersGrammar() : BaseGrammar(root_rule)
    {
#ifdef BOOST_HAS_LONG_LONG
        if (std::is_same<std::size_t, unsigned long long>::value)
            size_t_ = qi::ulong_long;
        else
            size_t_ = qi::ulong_;
#else
        size_t_ = qi::ulong_;
#endif

        timestamps_rule =
            qi::lit("timestamps=") >
            (qi::uint_ %
             ';')[ph::bind(&engine::api::MatchParameters::timestamps, qi::_r1) = qi::_1];

        // Add lambda functions for parsing yaw_rate and steering_angle
        const auto add_yaw_rate = [](engine::api::MatchParameters &match_parameters,
                                     boost::optional<double> yaw_value) {
            match_parameters.yaw_rate.push_back(yaw_value ? std::make_optional(engine::YawRate{*yaw_value}) : std::nullopt);
        };

        const auto add_steering_angle = [](engine::api::MatchParameters &match_parameters,
                                           boost::optional<double> angle_value) {
            match_parameters.steering_angle.push_back(angle_value ? std::make_optional(engine::SteeringAngle{*angle_value}) : std::nullopt);
        };

        // Add rules for yaw_rate and steering_angle
        yaw_rate_rule =
            qi::lit("yaw_rate=") >
            (-qi::double_)[ph::bind(add_yaw_rate, qi::_r1, qi::_1)] % ';';

        steering_angle_rule =
            qi::lit("steering_angle=") >
            (-qi::double_)[ph::bind(add_steering_angle, qi::_r1, qi::_1)] % ';';

        gaps_type.add("split", engine::api::MatchParameters::GapsType::Split)(
            "ignore", engine::api::MatchParameters::GapsType::Ignore);

        root_rule =
            BaseGrammar::query_rule(qi::_r1) > BaseGrammar::format_rule(qi::_r1) >
            -('?' > (timestamps_rule(qi::_r1) | 
                     yaw_rate_rule(qi::_r1) |           // Add yaw_rate parsing
                     steering_angle_rule(qi::_r1) |     // Add steering_angle parsing
                     BaseGrammar::base_rule(qi::_r1) |
                     (qi::lit("gaps=") >
                      gaps_type[ph::bind(&engine::api::MatchParameters::gaps, qi::_r1) = qi::_1]) |
                     (qi::lit("tidy=") >
                      qi::bool_[ph::bind(&engine::api::MatchParameters::tidy, qi::_r1) = qi::_1])) %
                        '&');
    }

  private:
    qi::rule<Iterator, Signature> root_rule;
    qi::rule<Iterator, Signature> timestamps_rule;
    qi::rule<Iterator, Signature> yaw_rate_rule;      // Add rule declaration
    qi::rule<Iterator, Signature> steering_angle_rule; // Add rule declaration
    qi::rule<Iterator, std::size_t()> size_t_;

    qi::symbols<char, engine::api::MatchParameters::GapsType> gaps_type;
};
} // namespace osrm::server::api

#endif