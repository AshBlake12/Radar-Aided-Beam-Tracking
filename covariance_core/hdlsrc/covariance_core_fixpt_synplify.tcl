project -new covariance_core_fixpt.prj
add_file covariance_core_fixpt_tc.v
add_file covariance_core_fixptp44.v
add_file covariance_core_fixpt_enb_bypass.v
add_file covariance_core_fixpt.v
set_option -technology VIRTEX4
set_option -part XC4VSX35
set_option -synthesis_onoff_pragma 0
set_option -frequency auto
project -run synthesis
