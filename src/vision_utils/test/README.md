# Vision Utils Test Suite

This directory contains comprehensive unit tests for the vision_utils package, specifically testing the ConfigLoader functionality.

## Test Files

### test_config_loader.cpp
Main test suite that validates the core configuration loading functionality:
- **LoadValidCameraConfig**: Tests loading valid camera configurations with all required fields
- **LoadSecondValidCameraConfig**: Tests loading multiple camera configurations
- **NonExistentCameraSerial**: Tests handling of non-existent camera serial numbers
- **LegacyConfigFormat**: Tests backward compatibility with legacy string-based configuration format
- **InvalidJsonFile**: Tests error handling for malformed JSON files
- **NonExistentConfigFile**: Tests behavior when configuration file doesn't exist
- **MissingRequiredFields**: Tests validation of required configuration fields
- **ConfigurationCaching**: Tests that configurations are properly cached
- **ForceReload**: Tests the reload functionality for refreshing configuration
- **EmptyConfigFile**: Tests handling of empty configuration files

### test_config_loader_utilities.cpp
Utility function tests that validate helper methods:
- **FormatStringToFourCC**: Tests conversion of format strings to OpenCV FourCC codes
- **ApiStringToCode**: Tests conversion of API preference strings to OpenCV codes
- **CaseSensitivity**: Tests case sensitivity handling in format and API conversions
- **EdgeCases**: Tests edge cases like empty strings, long strings, and special characters

## Test Coverage

The test suite provides comprehensive coverage for:
- Configuration file parsing (valid, invalid, missing)
- Camera configuration validation (required fields, optional fields)
- Error handling and graceful degradation
- Utility function conversions
- Caching and reload functionality
- Backward compatibility support

## Running Tests

To run all vision_utils tests:
```bash
colcon test --packages-select vision_utils
```

To run individual test executables:
```bash
./build/vision_utils/test_config_loader
./build/vision_utils/test_config_loader_utilities
```

## Test Results

All tests should pass, validating that the ConfigLoader:
1. Properly loads and validates camera configurations
2. Handles error conditions gracefully
3. Maintains backward compatibility
4. Provides reliable utility functions
5. Implements proper caching mechanisms

Total: 14 tests across 2 test suites, all passing.
