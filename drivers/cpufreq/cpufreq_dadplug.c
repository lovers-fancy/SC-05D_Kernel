/*
 * CPUFreq DadPlug governor
 *
 *
 * Based on hotplug governor
 * Copyright (C) 2010 Texas Instruments, Inc.
 *   Mike Turquette <mturquette@ti.com>
 *   Santosh Shilimkar <santosh.shilimkar@ti.com>
 *
 * Based on ondemand governor
 * Copyright (C)  2001 Russell King
 *           (C)  2003 Venkatesh Pallipadi <venkatesh.pallipadi@intel.com>,
 *                     Jun Nakajima <jun.nakajima@intel.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * Modified for early suspend support and tweaks by imoseyon (imoseyon@gmail.com)
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/cpufreq.h>
#include <linux/cpu.h>
#include <linux/jiffies.h>
#include <linux/kernel_stat.h>
#include <linux/mutex.h>
#include <linux/hrtimer.h>
#include <linux/tick.h>
#include <linux/ktime.h>
#include <linux/sched.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/earlysuspend.h>

// used for suspend code
static unsigned int susp_enabled = 0;
static unsigned int suspended = 0;
static unsigned int registration = 0;
static unsigned int suspend_cpu_up = 95;
static unsigned int highfreq = 1350000;

/* greater than 80% avg load across online CPUs increases frequency */
#define DEFAULT_UP_FREQ_MIN_LOAD			(80)

/* Keep 10% of idle under the up threshold when decreasing the frequency */
#define DEFAULT_FREQ_DOWN_DIFFERENTIAL			(10)

/* less than 30% avg load across online CPUs decreases frequency */
#define DEFAULT_DOWN_FREQ_MAX_LOAD			(30)

/* default sampling period (uSec) is bogus; 10x ondemand's default for x86 */
#define DEFAULT_SAMPLING_PERIOD				(100000)

/* default number of sampling periods to average before hotplug-in decision */
#define DEFAULT_HOTPLUG_IN_SAMPLING_PERIODS		(5)

/* default number of sampling periods to average before hotplug-out decision */
#define DEFAULT_HOTPLUG_OUT_SAMPLING_PERIODS		(20)

static void do_dadplug_timer(struct work_struct *work);
static int cpufreq_governor_dadplug(struct cpufreq_policy *policy,
		unsigned int event);
//static int hotplug_boost(struct cpufreq_policy *policy);

#ifndef CONFIG_CPU_FREQ_DEFAULT_GOV_DADPLUG
static
#endif
struct cpufreq_governor cpufreq_gov_dadplug = {
       .name                   = "dadplug",
       .governor               = cpufreq_governor_dadplug,
       .owner                  = THIS_MODULE,
};

struct cpu_dadplug_info_s {
	cputime64_t prev_cpu_idle;
	cputime64_t prev_cpu_wall;
	cputime64_t prev_cpu_nice;
	struct cpufreq_policy *cur_policy;
	struct delayed_work work;
	struct work_struct cpu_up_work;
	struct work_struct cpu_down_work;
	struct cpufreq_frequency_table *freq_table;
	int cpu;
	unsigned int boost_applied;
	/*
	 * percpu mutex that serializes governor limit change with
	 * do_dadplug_timer invocation. We do not want do_dadplug_timer to run
	 * when user is changing the governor or limits.
	 */
	struct mutex timer_mutex;
};
static DEFINE_PER_CPU(struct cpu_dadplug_info_s, hp_cpu_dadplug_info);

static unsigned int dadplug_enable;	/* number of CPUs using this policy */

/*
 * dadplug_mutex protects data in dadplug_tuners_ins from concurrent changes on
 * different CPUs. It protects dadplug_enable in governor start/stop.
 */
static DEFINE_MUTEX(dadplug_mutex);

static struct workqueue_struct	*khotplug_wq;

static struct dadplug_tuners {
	unsigned int sampling_rate;
	unsigned int up_threshold;
	unsigned int down_differential;
	unsigned int down_threshold;
	unsigned int hotplug_in_sampling_periods;
	unsigned int hotplug_out_sampling_periods;
	unsigned int hotplug_load_index;
	unsigned int *hotplug_load_history;
	unsigned int ignore_nice;
	unsigned int io_is_busy;
	unsigned int boost_timeout;
} dadplug_tuners_ins = {
	.sampling_rate =		DEFAULT_SAMPLING_PERIOD,
	.up_threshold =			DEFAULT_UP_FREQ_MIN_LOAD,
	.down_differential =            DEFAULT_FREQ_DOWN_DIFFERENTIAL,
	.down_threshold =		DEFAULT_DOWN_FREQ_MAX_LOAD,
	.hotplug_in_sampling_periods =	DEFAULT_HOTPLUG_IN_SAMPLING_PERIODS,
	.hotplug_out_sampling_periods =	DEFAULT_HOTPLUG_OUT_SAMPLING_PERIODS,
	.hotplug_load_index =		0,
	.ignore_nice =			0,
	.io_is_busy =			1,
	.boost_timeout = 		0,
};

/*
 * A corner case exists when switching io_is_busy at run-time: comparing idle
 * times from a non-io_is_busy period to an io_is_busy period (or vice-versa)
 * will misrepresent the actual change in system idleness.  We ignore this
 * corner case: enabling io_is_busy might cause freq increase and disabling
 * might cause freq decrease, which probably matches the original intent.
 */
static inline cputime64_t get_cpu_idle_time(unsigned int cpu, cputime64_t *wall)
{
        u64 idle_time;
        u64 iowait_time;

        /* cpufreq-dadplug always assumes CONFIG_NO_HZ */
        idle_time = get_cpu_idle_time_us(cpu, wall);

	/* add time spent doing I/O to idle time */
        if (dadplug_tuners_ins.io_is_busy) {
                iowait_time = get_cpu_iowait_time_us(cpu, wall);
                /* cpufreq-dadplug always assumes CONFIG_NO_HZ */
                if (iowait_time != -1ULL && idle_time >= iowait_time)
                        idle_time -= iowait_time;
        }

        return idle_time;
}

/************************** sysfs interface ************************/

/* XXX look at global sysfs macros in cpufreq.h, can those be used here? */

/* cpufreq_dadplug Governor Tunables */
#define show_one(file_name, object)					\
static ssize_t show_##file_name						\
(struct kobject *kobj, struct attribute *attr, char *buf)		\
{									\
	return sprintf(buf, "%u\n", dadplug_tuners_ins.object);		\
}
show_one(sampling_rate, sampling_rate);
show_one(up_threshold, up_threshold);
show_one(down_differential, down_differential);
show_one(down_threshold, down_threshold);
show_one(hotplug_in_sampling_periods, hotplug_in_sampling_periods);
show_one(hotplug_out_sampling_periods, hotplug_out_sampling_periods);
show_one(ignore_nice_load, ignore_nice);
show_one(io_is_busy, io_is_busy);
show_one(boost_timeout, boost_timeout);

static ssize_t store_boost_timeout(struct kobject *a, struct attribute *b,
				   const char *buf, size_t count)
{
	unsigned int input;
	int ret;
	ret = sscanf(buf, "%u", &input);
	if (ret != 1)
		return -EINVAL;

	mutex_lock(&dadplug_mutex);
	dadplug_tuners_ins.boost_timeout = input;
	mutex_unlock(&dadplug_mutex);

	return count;
}

static ssize_t store_sampling_rate(struct kobject *a, struct attribute *b,
				   const char *buf, size_t count)
{
	unsigned int input;
	int ret;
	ret = sscanf(buf, "%u", &input);
	if (ret != 1)
		return -EINVAL;

	mutex_lock(&dadplug_mutex);
	dadplug_tuners_ins.sampling_rate = input;
	mutex_unlock(&dadplug_mutex);

	return count;
}

static ssize_t store_up_threshold(struct kobject *a, struct attribute *b,
				  const char *buf, size_t count)
{
	unsigned int input;
	int ret;
	ret = sscanf(buf, "%u", &input);

	if (ret != 1 || input <= dadplug_tuners_ins.down_threshold) {
		return -EINVAL;
	}

	mutex_lock(&dadplug_mutex);
	dadplug_tuners_ins.up_threshold = input;
	mutex_unlock(&dadplug_mutex);

	return count;
}

static ssize_t store_down_differential(struct kobject *a, struct attribute *b,
				  const char *buf, size_t count)
{
	unsigned int input;
	int ret;
	ret = sscanf(buf, "%u", &input);

	if (ret != 1 || input >= dadplug_tuners_ins.up_threshold)
		return -EINVAL;

	mutex_lock(&dadplug_mutex);
	dadplug_tuners_ins.down_differential = input;
	mutex_unlock(&dadplug_mutex);

	return count;
}

static ssize_t store_down_threshold(struct kobject *a, struct attribute *b,
				  const char *buf, size_t count)
{
	unsigned int input;
	int ret;
	ret = sscanf(buf, "%u", &input);

	if (ret != 1 || input >= dadplug_tuners_ins.up_threshold) {
		return -EINVAL;
	}

	mutex_lock(&dadplug_mutex);
	dadplug_tuners_ins.down_threshold = input;
	mutex_unlock(&dadplug_mutex);

	return count;
}

static ssize_t store_hotplug_in_sampling_periods(struct kobject *a,
		struct attribute *b, const char *buf, size_t count)
{
	unsigned int input;
	unsigned int *temp;
	unsigned int max_windows;
	int ret;
	ret = sscanf(buf, "%u", &input);

	if (ret != 1)
		return -EINVAL;

	/* already using this value, bail out */
	if (input == dadplug_tuners_ins.hotplug_in_sampling_periods)
		return count;

	mutex_lock(&dadplug_mutex);
	ret = count;
	max_windows = max(dadplug_tuners_ins.hotplug_in_sampling_periods,
			dadplug_tuners_ins.hotplug_out_sampling_periods);

	/* no need to resize array */
	if (input <= max_windows) {
		dadplug_tuners_ins.hotplug_in_sampling_periods = input;
		goto out;
	}

	/* resize array */
	temp = kmalloc((sizeof(unsigned int) * input), GFP_KERNEL);

	if (!temp || IS_ERR(temp)) {
		ret = -ENOMEM;
		goto out;
	}

	memcpy(temp, dadplug_tuners_ins.hotplug_load_history,
			(max_windows * sizeof(unsigned int)));
	kfree(dadplug_tuners_ins.hotplug_load_history);

	/* replace old buffer, old number of sampling periods & old index */
	dadplug_tuners_ins.hotplug_load_history = temp;
	dadplug_tuners_ins.hotplug_in_sampling_periods = input;
	dadplug_tuners_ins.hotplug_load_index = max_windows;
out:
	mutex_unlock(&dadplug_mutex);

	return ret;
}

static ssize_t store_hotplug_out_sampling_periods(struct kobject *a,
		struct attribute *b, const char *buf, size_t count)
{
	unsigned int input;
	unsigned int *temp;
	unsigned int max_windows;
	int ret;
	ret = sscanf(buf, "%u", &input);

	if (ret != 1)
		return -EINVAL;

	/* already using this value, bail out */
	if (input == dadplug_tuners_ins.hotplug_out_sampling_periods)
		return count;

	mutex_lock(&dadplug_mutex);
	ret = count;
	max_windows = max(dadplug_tuners_ins.hotplug_in_sampling_periods,
			dadplug_tuners_ins.hotplug_out_sampling_periods);

	/* no need to resize array */
	if (input <= max_windows) {
		dadplug_tuners_ins.hotplug_out_sampling_periods = input;
		goto out;
	}

	/* resize array */
	temp = kmalloc((sizeof(unsigned int) * input), GFP_KERNEL);

	if (!temp || IS_ERR(temp)) {
		ret = -ENOMEM;
		goto out;
	}

	memcpy(temp, dadplug_tuners_ins.hotplug_load_history,
			(max_windows * sizeof(unsigned int)));
	kfree(dadplug_tuners_ins.hotplug_load_history);

	/* replace old buffer, old number of sampling periods & old index */
	dadplug_tuners_ins.hotplug_load_history = temp;
	dadplug_tuners_ins.hotplug_out_sampling_periods = input;
	dadplug_tuners_ins.hotplug_load_index = max_windows;
out:
	mutex_unlock(&dadplug_mutex);

	return ret;
}

static ssize_t store_ignore_nice_load(struct kobject *a, struct attribute *b,
				      const char *buf, size_t count)
{
	unsigned int input;
	int ret;

	unsigned int j;

	ret = sscanf(buf, "%u", &input);
	if (ret != 1)
		return -EINVAL;

	if (input > 1)
		input = 1;

	mutex_lock(&dadplug_mutex);
	if (input == dadplug_tuners_ins.ignore_nice) { /* nothing to do */
		mutex_unlock(&dadplug_mutex);
		return count;
	}
	dadplug_tuners_ins.ignore_nice = input;

	/* we need to re-evaluate prev_cpu_idle */
	for_each_online_cpu(j) {
		struct cpu_dadplug_info_s *dadplug_info;
		dadplug_info = &per_cpu(hp_cpu_dadplug_info, j);
		dadplug_info->prev_cpu_idle = get_cpu_idle_time(j,
						&dadplug_info->prev_cpu_wall);
		if (dadplug_tuners_ins.ignore_nice)
			dadplug_info->prev_cpu_nice = kstat_cpu(j).cpustat.nice;

	}
	mutex_unlock(&dadplug_mutex);

	return count;
}

static ssize_t store_io_is_busy(struct kobject *a, struct attribute *b,
				   const char *buf, size_t count)
{
	unsigned int input;
	int ret;

	ret = sscanf(buf, "%u", &input);
	if (ret != 1)
		return -EINVAL;

	mutex_lock(&dadplug_mutex);
	dadplug_tuners_ins.io_is_busy = !!input;
	mutex_unlock(&dadplug_mutex);

	return count;
}

define_one_global_rw(sampling_rate);
define_one_global_rw(up_threshold);
define_one_global_rw(down_differential);
define_one_global_rw(down_threshold);
define_one_global_rw(hotplug_in_sampling_periods);
define_one_global_rw(hotplug_out_sampling_periods);
define_one_global_rw(ignore_nice_load);
define_one_global_rw(io_is_busy);
define_one_global_rw(boost_timeout);

static struct attribute *dadplug_attributes[] = {
	&sampling_rate.attr,
	&up_threshold.attr,
	&down_differential.attr,
	&down_threshold.attr,
	&hotplug_in_sampling_periods.attr,
	&hotplug_out_sampling_periods.attr,
	&ignore_nice_load.attr,
	&io_is_busy.attr,
	&boost_timeout.attr,
	NULL
};

static struct attribute_group dadplug_attr_group = {
	.attrs = dadplug_attributes,
	.name = "dadplug",
};

/************************** sysfs end ************************/

static void dadplug_check_cpu(struct cpu_dadplug_info_s *this_dadplug_info)
{
	/* combined load of all enabled CPUs */
	unsigned int total_load = 0;
	/* single largest CPU load percentage*/
	unsigned int max_load = 0;
	/* largest CPU load in terms of frequency */
	unsigned int max_load_freq = 0;
	/* average load across all enabled CPUs */
	unsigned int avg_load = 0;
	/* average load across multiple sampling periods for hotplug events */
	unsigned int hotplug_in_avg_load = 0;
	unsigned int hotplug_out_avg_load = 0;
	/* number of sampling periods averaged for hotplug decisions */
	unsigned int periods;

	struct cpufreq_policy *policy;
	unsigned int i, j;

	policy = this_dadplug_info->cur_policy;

	/*
	 * cpu load accounting
	 * get highest load, total load and average load across all CPUs
	 */
	for_each_cpu(j, policy->cpus) {
		unsigned int load;
		unsigned int idle_time, wall_time;
		cputime64_t cur_wall_time, cur_idle_time;
		struct cpu_dadplug_info_s *j_dadplug_info;

		j_dadplug_info = &per_cpu(hp_cpu_dadplug_info, j);

		/* update both cur_idle_time and cur_wall_time */
		cur_idle_time = get_cpu_idle_time(j, &cur_wall_time);

		/* how much wall time has passed since last iteration? */
		wall_time = (unsigned int) cputime64_sub(cur_wall_time,
				j_dadplug_info->prev_cpu_wall);
		j_dadplug_info->prev_cpu_wall = cur_wall_time;

		/* how much idle time has passed since last iteration? */
		idle_time = (unsigned int) cputime64_sub(cur_idle_time,
				j_dadplug_info->prev_cpu_idle);
		j_dadplug_info->prev_cpu_idle = cur_idle_time;

		if (unlikely(!wall_time || wall_time < idle_time))
			continue;

		/* load is the percentage of time not spent in idle */
		load = 100 * (wall_time - idle_time) / wall_time;

		/* keep track of combined load across all CPUs */
		total_load += load;

		/* keep track of highest single load across all CPUs */
		if (load > max_load)
			max_load = load;
	}

	/* use the max load in the OPP freq change policy */
	max_load_freq = max_load * policy->cur;

	/* calculate the average load across all related CPUs */
	avg_load = total_load / num_online_cpus();

	mutex_lock(&dadplug_mutex);

	/*
	 * hotplug load accounting
	 * average load over multiple sampling periods
	 */

	/* how many sampling periods do we use for hotplug decisions? */
	periods = max(dadplug_tuners_ins.hotplug_in_sampling_periods,
			dadplug_tuners_ins.hotplug_out_sampling_periods);

	/* store avg_load in the circular buffer */
	dadplug_tuners_ins.hotplug_load_history[dadplug_tuners_ins.hotplug_load_index]
		= avg_load;

	/* compute average load across in & out sampling periods */
	for (i = 0, j = dadplug_tuners_ins.hotplug_load_index;
			i < periods; i++, j--) {
		if (i < dadplug_tuners_ins.hotplug_in_sampling_periods)
			hotplug_in_avg_load +=
				dadplug_tuners_ins.hotplug_load_history[j];
		if (i < dadplug_tuners_ins.hotplug_out_sampling_periods)
			hotplug_out_avg_load +=
				dadplug_tuners_ins.hotplug_load_history[j];

		if (j == 0)
			j = periods;
	}

	hotplug_in_avg_load = hotplug_in_avg_load /
		dadplug_tuners_ins.hotplug_in_sampling_periods;

	hotplug_out_avg_load = hotplug_out_avg_load /
		dadplug_tuners_ins.hotplug_out_sampling_periods;

	/* return to first element if we're at the circular buffer's end */
	if (++dadplug_tuners_ins.hotplug_load_index == periods)
		dadplug_tuners_ins.hotplug_load_index = 0;

	/* check if auxiliary CPU is needed based on avg_load */
	/* imoseyon - support cpu_up even if suspended if load is super high */
	if (!suspended || (suspended && avg_load > suspend_cpu_up))
		if (avg_load > dadplug_tuners_ins.up_threshold) {
		/* should we enable auxillary CPUs? */
		if (num_online_cpus() < num_possible_cpus() && hotplug_in_avg_load >
				dadplug_tuners_ins.up_threshold) {
			queue_work_on(this_dadplug_info->cpu, khotplug_wq,
					&this_dadplug_info->cpu_up_work);
			goto out;
		}
	}

	/* check for frequency increase based on max_load */
	if (max_load > dadplug_tuners_ins.up_threshold) {
		// imosey go to highest freq only if load is really high
		if (max_load < suspend_cpu_up && highfreq <= policy->max) 
			__cpufreq_driver_target(policy, highfreq,
				CPUFREQ_RELATION_H);
		else
		/* increase to highest frequency supported */
		if (policy->cur < policy->max)
			__cpufreq_driver_target(policy, policy->max,
					CPUFREQ_RELATION_H);

		goto out;
	}

	/* check for frequency decrease */
	if (avg_load < dadplug_tuners_ins.down_threshold) {
		/* are we at the minimum frequency already? */
		if (policy->cur <= policy->min) {
			/* should we disable auxillary CPUs? */
			if (num_online_cpus() > 1 && hotplug_out_avg_load <
					dadplug_tuners_ins.down_threshold) {
				queue_work_on(this_dadplug_info->cpu, khotplug_wq,
					&this_dadplug_info->cpu_down_work);
			}
			goto out;
		}
	}

	/*
	 * go down to the lowest frequency which can sustain the load by
	 * keeping 30% of idle in order to not cross the up_threshold
	 */
	if ((max_load_freq <
	    (dadplug_tuners_ins.up_threshold - dadplug_tuners_ins.down_differential) *
	     policy->cur) && (policy->cur > policy->min)) {
		unsigned int freq_next;
		freq_next = max_load_freq /
				(dadplug_tuners_ins.up_threshold -
				 dadplug_tuners_ins.down_differential);

		if (freq_next < policy->min)
			freq_next = policy->min;

		 __cpufreq_driver_target(policy, freq_next,
					 CPUFREQ_RELATION_L);
	}
out:
	mutex_unlock(&dadplug_mutex);
	return;
}

static void do_cpu_up(struct work_struct *work)
{
	int i = num_online_cpus();
	if( i < num_possible_cpus() && !cpu_online(i) ) cpu_up(i);
}

static void do_cpu_down(struct work_struct *work)
{
	int i = num_online_cpus() - 1;
	if( i > 0 && cpu_online(i) ) cpu_down(i);
}

static void do_dadplug_timer(struct work_struct *work)
{
	struct cpu_dadplug_info_s *dadplug_info =
		container_of(work, struct cpu_dadplug_info_s, work.work);
	unsigned int cpu = dadplug_info->cpu;
	int delay = 0;

	mutex_lock(&dadplug_info->timer_mutex);
	if (!dadplug_info->boost_applied) {
	    dadplug_check_cpu(dadplug_info);
		/* We want all related CPUs to do sampling nearly on same jiffy */
		delay = usecs_to_jiffies(dadplug_tuners_ins.sampling_rate);
	} else {
		delay = usecs_to_jiffies(dadplug_tuners_ins.boost_timeout);
		dadplug_info->boost_applied = 0;
		if (num_online_cpus() < 2)
			queue_work_on(cpu, khotplug_wq,
						&dadplug_info->cpu_up_work);
	}
	queue_delayed_work_on(cpu, khotplug_wq, &dadplug_info->work, delay);
	mutex_unlock(&dadplug_info->timer_mutex);
}

static inline void dadplug_timer_init(struct cpu_dadplug_info_s *dadplug_info)
{
	/* We want all related CPUs to do sampling nearly on same jiffy */
	int delay = usecs_to_jiffies(dadplug_tuners_ins.sampling_rate);
	delay -= jiffies % delay;

	INIT_DELAYED_WORK_DEFERRABLE(&dadplug_info->work, do_dadplug_timer);
	INIT_WORK(&dadplug_info->cpu_up_work, do_cpu_up);
	INIT_WORK(&dadplug_info->cpu_down_work, do_cpu_down);
	if (!dadplug_info->boost_applied)
		delay = usecs_to_jiffies(dadplug_tuners_ins.boost_timeout);
	queue_delayed_work_on(dadplug_info->cpu, khotplug_wq, &dadplug_info->work,
		delay);
}

static inline void dadplug_timer_exit(struct cpu_dadplug_info_s *dadplug_info)
{
	cancel_delayed_work_sync(&dadplug_info->work);
}

static void hotplug_suspend(int suspend)
{
	if (!susp_enabled) return;
			if (!suspend) {
		suspended = 0;
		if (num_online_cpus() < 2) cpu_up(1);
		pr_info("[PaPa] dadplug awake cpu1 up\n");
	} else {
		suspended = 1;
			if (num_online_cpus() > 1) cpu_down(1);
		pr_info("[PaPa] dadplug suspended cpu1 down\n");
	}
}
static void hotplug_early_suspend(struct early_suspend *handler) {
	if (!registration) hotplug_suspend(1);
}

static void hotplug_late_resume(struct early_suspend *handler) {
	hotplug_suspend(0);
}

static struct early_suspend hotplug_power_suspend = {
	.suspend = hotplug_early_suspend,
	.resume = hotplug_late_resume,
	.level = EARLY_SUSPEND_LEVEL_DISABLE_FB + 1,
};

static int cpufreq_governor_dadplug(struct cpufreq_policy *policy,
				   unsigned int event)
{
	unsigned int cpu = policy->cpu;
	struct cpu_dadplug_info_s *this_dadplug_info;
	unsigned int i, j, max_periods;
	int rc;

	this_dadplug_info = &per_cpu(hp_cpu_dadplug_info, cpu);

	switch (event) {
	case CPUFREQ_GOV_START:
		if ((!cpu_online(cpu)) || (!policy->cur))
			return -EINVAL;

		mutex_lock(&dadplug_mutex);
		dadplug_enable++;
		for_each_cpu(j, policy->cpus) {
			struct cpu_dadplug_info_s *j_dadplug_info;
			j_dadplug_info = &per_cpu(hp_cpu_dadplug_info, j);
			j_dadplug_info->cur_policy = policy;

			j_dadplug_info->prev_cpu_idle = get_cpu_idle_time(j,
						&j_dadplug_info->prev_cpu_wall);
			if (dadplug_tuners_ins.ignore_nice)
				j_dadplug_info->prev_cpu_nice =
						kstat_cpu(j).cpustat.nice;

			max_periods = max(DEFAULT_HOTPLUG_IN_SAMPLING_PERIODS,
					DEFAULT_HOTPLUG_OUT_SAMPLING_PERIODS);
			dadplug_tuners_ins.hotplug_load_history = kmalloc(
					(sizeof(unsigned int) * max_periods),
					GFP_KERNEL);
			if (!dadplug_tuners_ins.hotplug_load_history) {
				WARN_ON(1);
				return -ENOMEM;
			}
			for (i = 0; i < max_periods; i++)
				dadplug_tuners_ins.hotplug_load_history[i] = 50;
		}
		this_dadplug_info->cpu = cpu;
		this_dadplug_info->freq_table = cpufreq_frequency_get_table(cpu);
		/*
		 * Start the timerschedule work, when this governor
		 * is used for first time
		 */
		if (dadplug_enable == 1) {
			rc = sysfs_create_group(cpufreq_global_kobject,
						&dadplug_attr_group);
			if (rc) {
				mutex_unlock(&dadplug_mutex);
				return rc;
			}
		}
		if (!dadplug_tuners_ins.boost_timeout)
			dadplug_tuners_ins.boost_timeout =  dadplug_tuners_ins.sampling_rate * 15;
		mutex_unlock(&dadplug_mutex);

		mutex_init(&this_dadplug_info->timer_mutex);
		dadplug_timer_init(this_dadplug_info);
		susp_enabled = 1;
		registration = 1;
			register_early_suspend(&hotplug_power_suspend);
		registration = 0;
			pr_info("[PaPa] dadplug start\n");
		break;

	case CPUFREQ_GOV_STOP:
		dadplug_timer_exit(this_dadplug_info);

		mutex_lock(&dadplug_mutex);
		mutex_destroy(&this_dadplug_info->timer_mutex);
		dadplug_enable--;
		mutex_unlock(&dadplug_mutex);
		if (!dadplug_enable)
			sysfs_remove_group(cpufreq_global_kobject,
					   &dadplug_attr_group);
		kfree(dadplug_tuners_ins.hotplug_load_history);
		/*
		 * XXX BIG CAVEAT: Stopping the governor with CPU1 offline
		 * will result in it remaining offline until the user onlines
		 * it again.  It is up to the user to do this (for now).
		 */
			susp_enabled = 0;
				unregister_early_suspend(&hotplug_power_suspend);
				pr_info("[PaPa] dadplug inactive\n");
		break;

	case CPUFREQ_GOV_LIMITS:
		mutex_lock(&this_dadplug_info->timer_mutex);
		if (policy->max < this_dadplug_info->cur_policy->cur)
			__cpufreq_driver_target(this_dadplug_info->cur_policy,
				policy->max, CPUFREQ_RELATION_H);
		else if (policy->min > this_dadplug_info->cur_policy->cur)
			__cpufreq_driver_target(this_dadplug_info->cur_policy,
				policy->min, CPUFREQ_RELATION_L);
		mutex_unlock(&this_dadplug_info->timer_mutex);
		break;
	}
	return 0;
}

#if 0
static int hotplug_boost(struct cpufreq_policy *policy)
{
	unsigned int cpu = policy->cpu;
	struct cpu_dadplug_info_s *this_dadplug_info;

	this_dadplug_info = &per_cpu(hp_cpu_dadplug_info, cpu);

#if 0
	/* Already at max? */
	if (policy->cur == policy->max)
		return;
#endif

	mutex_lock(&this_dadplug_info->timer_mutex);
	this_dadplug_info->boost_applied = 1;
	__cpufreq_driver_target(policy, policy->max,
		CPUFREQ_RELATION_H);
	mutex_unlock(&this_dadplug_info->timer_mutex);

	return 0;
}
#endif

static int __init cpufreq_gov_dadplug_init(void)
{
	int err;
	cputime64_t wall;
	u64 idle_time;
	int cpu = get_cpu();

	idle_time = get_cpu_idle_time_us(cpu, &wall);
	put_cpu();
	if (idle_time != -1ULL) {
		dadplug_tuners_ins.up_threshold = DEFAULT_UP_FREQ_MIN_LOAD;
	} else {
		pr_err("cpufreq-dadplug: %s: assumes CONFIG_NO_HZ\n",
				__func__);
		return -EINVAL;
	}

	khotplug_wq = create_workqueue("khotplug");
	if (!khotplug_wq) {
		pr_err("Creation of khotplug failed\n");
		return -EFAULT;
	}
	err = cpufreq_register_governor(&cpufreq_gov_dadplug);
	if (err)
		destroy_workqueue(khotplug_wq);

	return err;
}

//static void __exit cpufreq_gov_dbs_exit(void)
static void __exit cpufreq_gov_dadplug_exit(void)
{
	cpufreq_unregister_governor(&cpufreq_gov_dadplug);
	destroy_workqueue(khotplug_wq);
}

MODULE_DESCRIPTION("'cpufreq_dadplug' - cpufreq governor for dynamic frequency scaling and CPU hotplug");
MODULE_LICENSE("GPL");

#ifdef CONFIG_CPU_FREQ_DEFAULT_GOV_DADPLUG
fs_initcall(cpufreq_gov_dadplug_init);
#else
module_init(cpufreq_gov_dadplug_init);
#endif
module_exit(cpufreq_gov_dadplug_exit);
