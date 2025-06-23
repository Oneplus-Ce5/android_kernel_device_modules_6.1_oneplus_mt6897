load("//build/kernel/oplus:oplus_modules_define.bzl", "define_oplus_ddk_module")
load("//build/kernel/oplus:oplus_modules_dist.bzl", "ddk_copy_to_dist_dir")

def define_oplus_local_modules():

    define_oplus_ddk_module(
        name = "oplus_bsp_game_opt",
        srcs = native.glob([
            "**/*.h",
            "cpu_load.c",
            "cpufreq_limits.c",
            "debug.c",
            "early_detect.c",
            "fake_cpufreq.c",
            "game_ctrl.c",
            "rt_info.c",
            "task_util.c",
            "yield_opt.c",
            "frame_load.c",
            "frame_sync.c",
            "task_boost/heavy_task_boost.c",
            "task_boost/boost_proc.c",
            "critical_task_boost.c",
        ]),
        conditional_srcs = {
        },
        includes = ["."],
        copts = select({
            "//build/kernel/kleaf:kocov_is_true": ["-fprofile-arcs", "-ftest-coverage"],
            "//conditions:default": [],
        }),
    )

    ddk_copy_to_dist_dir(
        name = "oplus_bsp_game",
        module_list = [
            "oplus_bsp_game_opt",
        ],
    )
