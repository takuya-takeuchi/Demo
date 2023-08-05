import Foundation

let CPUINFO_PACKAGE_NAME_MAX = 48

struct CpuinfoPackage{
    var name: (Int8, Int8, Int8, Int8, Int8, Int8, Int8, Int8, Int8, Int8, Int8, Int8, Int8, Int8, Int8, Int8, Int8, Int8, Int8, Int8, Int8, Int8, Int8, Int8, Int8, Int8, Int8, Int8, Int8, Int8, Int8, Int8, Int8, Int8, Int8, Int8, Int8, Int8, Int8, Int8, Int8, Int8, Int8, Int8, Int8, Int8, Int8, Int8)
    var processor_start: UInt32
    var processor_count: UInt32
    var core_start: UInt32
    var core_count: UInt32
    var cluster_start: UInt32
    var cluster_count: UInt32
}