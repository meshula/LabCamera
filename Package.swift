// swift-tools-version:5.3
// The swift-tools-version declares the minimum version of Swift required to build this package.

import PackageDescription

let package = Package(
    name: "LabCamera",
    platforms: [.macOS(.v10_15), .iOS(.v11)],
    products: [
        // Products define the executables and libraries a package produces, and make them visible to other packages.

        .library(
            name: "labcamera-swift",
            targets: ["LabCameraSwift"]
                ),
        .library(
            name: "labcamera-c",
            targets: ["liblabcamera"]
                ),
        .executable(
            name: "Sample-swift",
            targets: ["Sample"])
    ],
    dependencies: [
        // Dependencies declare other packages that this package depends on.
        // .package(url: /* package url */, from: "1.0.0"),
    ],
    targets: [
        // Targets are the basic building blocks of a package. A target can define a module or a test suite.
        // Targets can depend on other targets in this package, and on products in packages this package depends on.
        .testTarget(
            name: "labcamera-swift-tests",
            dependencies: ["LabCameraSwift"]),
        .target(name: "LabCameraSwift",
                dependencies: ["liblabcamera"],
                path: "swift"),
        .target(name: "liblabcamera",
                dependencies: [],
                path: "cpp"),
        .target(name: "Sample",
                dependencies:["LabCameraSwift"],
                path: "examples/swift")
    ],
    cxxLanguageStandard: CXXLanguageStandard.cxx1z
)
