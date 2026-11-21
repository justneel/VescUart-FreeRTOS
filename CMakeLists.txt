# ESP-IDF component. Add this repository to your project's components/ folder, or depend on it
# through idf_component.yml with a git URL.
idf_component_register(
    SRCS "src/VescUart.cpp" "src/VescIdfUart.cpp" "src/buffer.cpp" "src/crc.cpp"
    INCLUDE_DIRS "src"
    REQUIRES driver freertos esp_common log
)
