CSOURCES += uCOS-III/Source/os_core.c
CSOURCES += uCOS-III/Source/os_dbg.c
CSOURCES += uCOS-III/Source/os_flag.c
CSOURCES += uCOS-III/Source/os_int.c
CSOURCES += uCOS-III/Source/os_mem.c
CSOURCES += uCOS-III/Source/os_msg.c
CSOURCES += uCOS-III/Source/os_mutex.c
CSOURCES += uCOS-III/Source/os_pend_multi.c
CSOURCES += uCOS-III/Source/os_prio.c
CSOURCES += uCOS-III/Source/os_q.c
CSOURCES += uCOS-III/Source/os_sem.c
CSOURCES += uCOS-III/Source/os_stat.c
CSOURCES += uCOS-III/Source/os_task.c
CSOURCES += uCOS-III/Source/os_tick.c
CSOURCES += uCOS-III/Source/os_time.c
CSOURCES += uCOS-III/Source/os_tmr.c
CSOURCES += uCOS-III/Source/os_var.c
CSOURCES += uCOS-III/Source/os_cfg_app.c

CSOURCES += uCOS-III/Port/os_cpu_c.c
CSOURCES += uCOS-III/Port/cpu_core.c

ASOURCES += uCOS-III/Port/os_cpu_a.s

INCDIR   += -I./uCOS-III/Source
INCDIR   += -I./uCOS-III/Port
INCDIR   += -I./uCOS-III/Cfg