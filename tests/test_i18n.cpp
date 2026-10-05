#include "sim/core/I18n.hpp"

#include <catch2/catch_test_macros.hpp>

TEST_CASE("catalog falls back from Chinese to English") {
    sim::I18nCatalog catalog;
    std::string error;
    REQUIRE(catalog.loadJson("en", R"({"menu.file":"File","only.en":"English"})", error));
    REQUIRE(catalog.loadJson("zh_CN", R"({"menu.file":"文件"})", error));
    catalog.setLocale("zh_CN");
    REQUIRE(catalog.translate("menu.file") == "文件");
    REQUIRE(catalog.translate("only.en") == "English");
    REQUIRE(catalog.translate("missing.key") == "missing.key");
    catalog.setLocale("fr");
    REQUIRE(catalog.translate("menu.file") == "File");
}
