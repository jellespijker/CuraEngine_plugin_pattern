#ifndef CURAENGINE_PLUGIN_PATTERN_INCLUDE_INFILL_INFILL_H
#define CURAENGINE_PLUGIN_PATTERN_INCLUDE_INFILL_INFILL_H

#include "infill/hilbert.h"
#include "infill/pattern.h"

#include <fmt/format.h>

#include <memory>
#include <string_view>
#include <tl/expected.hpp>

namespace pattern
{

[[nodiscard]] tl::expected<std::unique_ptr<InfillPattern>, std::string> make_infill_generator(const std::string_view& pattern_name, auto&&... args) noexcept
{
    auto pattern = make_pattern(pattern_name);
    if (! pattern.has_value())
    {
        return tl::make_unexpected(pattern.error());
    }
    switch (pattern.value())
    {
    case Pattern::HILBERT:
        return std::make_unique<Hilbert>(std::forward<decltype(args)...>(args...));
    default:
        return tl::make_unexpected(fmt::format("Pattern {}, does not have a generator!", pattern_name));
    }
}


} // namespace pattern

#endif // CURAENGINE_PLUGIN_PATTERN_INCLUDE_INFILL_INFILL_H
