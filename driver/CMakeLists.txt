
generateFMU("driver"
        SOURCES
        "model.cpp"
        FMI_VERSIONS
        "fmi2;fmi3"
        WITH_SOURCES
)
