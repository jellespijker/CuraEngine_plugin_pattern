#ifndef CURAENGINE_PLUGIN_PATTERN_INCLUDE_INFILL_PATTERN_H
#define CURAENGINE_PLUGIN_PATTERN_INCLUDE_INFILL_PATTERN_H

#include <fmt/format.h>
#include <tl/expected.hpp>

#include <string_view>
#include <unordered_map>

namespace pattern
{

enum class Pattern : int
{
    HILBERT,
    // add more
};

[[nodiscard]] tl::expected<Pattern, std::string> make_pattern(const std::string_view& name) noexcept
{
    static std::unordered_map<std::string_view, Pattern> mapping{
        { "PLUGIN::CuraEnginePattern@0.1.0::HILBERT", Pattern::HILBERT },
    };
    if (! mapping.contains(name))
    {
        return tl::make_unexpected(fmt::format("Pattern: {} is unknown", name));
    }
    return mapping.at(name);
}

struct InfillPattern
{
    [[nodiscard]] virtual std::tuple<ClipperLib::Paths, ClipperLib::Paths> operator()(const ClipperLib::Paths& outlines) noexcept = 0;
};


} // namespace pattern

#endif // CURAENGINE_PLUGIN_PATTERN_INCLUDE_INFILL_PATTERN_H
