/****************************************************************************
 *
 *   Copyright (C) 2012 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file uORB.cpp
 * A lightweight object broker.
 */

#include <nuttx/config.h>

#include <drivers/device/device.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>
#include <fcntl.h>
#include <poll.h>
#include <errno.h>
#include <stdio.h>
#include <math.h>
#include <unistd.h>

#include <nuttx/arch.h>
#include <nuttx/wqueue.h>
#include <nuttx/clock.h>

#include <drivers/drv_hrt.h>

#include <drivers/drv_orb_dev.h>

#include "uORB.h"

/**
 * Utility functions.
 */
namespace
{

static const unsigned orb_maxpath = 64;

/* oddly, ERROR is not defined for c++ */
#ifdef ERROR
# undef ERROR
#endif
const int ERROR = -1;

enum Flavor {
	PUBSUB,
	PARAM
};

int
node_mkpath(char *buf, Flavor f, const struct orb_metadata *meta)
{
	unsigned len;

	len = snprintf(buf, orb_maxpath, "/%s/%s",
		       (f == PUBSUB) ? "obj" : "param",
		       meta->o_name);

	if (len >= orb_maxpath)
		return -ENAMETOOLONG;

	return OK;
}

}

/**
 * Per-object device instance.
 */
class ORBDevNode : public device::CDev
{
public:
	ORBDevNode(const struct orb_metadata *meta, const char *name, const char *path);
	~ORBDevNode();

	virtual int		open(struct file *filp);
	virtual int		close(struct file *filp);
	virtual ssize_t		read(struct file *filp, char *buffer, size_t buflen);
	virtual ssize_t		write(struct file *filp, const char *buffer, size_t buflen);
	virtual int		ioctl(struct file *filp, int cmd, unsigned long arg);

	static ssize_t		publish(const orb_metadata *meta, orb_advert_t handle, const void *data);

protected:
	virtual pollevent_t	poll_state(struct file *filp);
	virtual void		poll_notify_one(struct pollfd *fds, pollevent_t events);

private:
	struct SubscriberData {
		unsigned	generation;	/**< last generation the subscriber has seen */
		unsigned	update_interval; /**< if nonzero minimum interval between updates */
		struct hrt_call	update_call;	/**< deferred wakeup call if update_period is nonzero */
		void		*poll_priv;	/**< saved copy of fds->f_priv while poll is active */
		bool		update_reported; /**< true if we have reported the update via poll/check */
	};

	const struct orb_metadata *_meta;	/**< object metadata information */
	uint8_t			*_data;		/**< allocated object buffer */
	hrt_abstime		_last_update;	/**< time the object was last updated */
	volatile unsigned 	_generation;	/**< object generation count */
	pid_t			_publisher;	/**< if nonzero, current publisher */

	SubscriberData		*filp_to_sd(struct file *filp) {
		SubscriberData *sd = (SubscriberData *)(filp->f_priv);
		return sd;
	}

	/**
	 * Perform a deferred update for a rate-limited subscriber.
	 */
	void			update_deferred();

	/**
	 * Bridge from hrt_call to update_deferred
	 *
	 * void *arg		ORBDevNode pointer for which the deferred update is performed.
	 */
	static void		update_deferred_trampoline(void *arg);

	/**
	 * Check whether a topic appears updated to a subscriber.
	 *
	 * @param sd		The subscriber for whom to check.
	 * @return		True if the topic should appear updated to the subscriber
	 */
	bool			appears_updated(SubscriberData *sd);
};

ORBDevNode::ORBDevNode(const struct orb_metadata *meta, const char *name, const char *path) :
	CDev(name, path),
	_meta(meta),
	_data(nullptr),
	_last_update(0),
	_generation(0),
	_publisher(0)
{
	// enable debug() calls
	_debug_enabled = true;
}

ORBDevNode::~ORBDevNode()
{
	if (_data != nullptr)
		delete[] _data;
}

int
ORBDevNode::open(struct file *filp)
{
	int ret;

	/* is this a publisher? */
	if (filp->f_oflags == O_WRONLY) {

		/* become the publisher if we can */
		lock();

		if (_publisher == 0) {
			_publisher = getpid();
			ret = OK;

		} else {
			ret = -EBUSY;
		}

		unlock();

		/* now complete the open */
		if (ret == OK) {
			ret = CDev::open(filp);

			/* open failed - not the publisher anymore */
			if (ret != OK)
				_publisher = 0;
		}

		return ret;
	}

	/* is this a new subscriber? */
	if (filp->f_oflags == O_RDONLY) {

		/* allocate subscriber data */
		SubscriberData *sd = new SubscriberData;

		if (nullptr == sd)
			return -ENOMEM;

		memset(sd, 0, sizeof(*sd));

		/* default to no pending update */
		sd->generation = _generation;

		filp->f_priv = (void *)sd;

		ret = CDev::open(filp);

		if (ret != OK)
			free(sd);

		return ret;
	}

	/* can only be pub or sub, not both */
	return -EINVAL;
}

int
ORBDevNode::close(struct file *filp)
{
	/* is this the publisher closing? */
	if (getpid() == _publisher) {
		_publisher = 0;

	} else {
		SubscriberData *sd = filp_to_sd(filp);

		if (sd != nullptr) {
			hrt_cancel(&sd->update_call);
			delete sd;
		}
	}

	return CDev::close(filp);
}

ssize_t
ORBDevNode::read(struct file *filp, char *buffer, size_t buflen)
{
	SubscriberData *sd = (SubscriberData *)filp_to_sd(filp);

	/* if the object has not been written yet, return zero */
	if (_data == nullptr)
		return 0;

	/* if the caller's buffer is the wrong size, that's an error */
	if (buflen != _meta->o_size)
		return -EIO;

	/*
	 * Perform an atomic copy & state update
	 */
	irqstate_t flags = irqsave();

	/* if the caller doesn't want the data, don't give it to them */
	if (nullptr != buffer)
		memcpy(buffer, _data, _meta->o_size);

	/* track the last generation that the file has seen */
	sd->generation = _generation;

	/*
	 * Clear the flag that indicates that an update has been reported, as
	 * we have just collected it.
	 */
	sd->update_reported = false;

	irqrestore(flags);

	return _meta->o_size;
}

ssize_t
ORBDevNode::write(struct file *filp, const char *buffer, size_t buflen)
{
	/*
	 * Writes are legal from interrupt context as long as the
	 * object has already been initialised from thread context.
	 *
	 * Writes outside interrupt context will allocate the object
	 * if it has not yet been allocated.
	 *
	 * Note that filp will usually be NULL.
	 */
	if (nullptr == _data) {
		if (!up_interrupt_context()) {

			lock();

			/* re-check size */
			if (nullptr == _data)
				_data = new uint8_t[_meta->o_size];

			unlock();
		}

		/* failed or could not allocate */
		if (nullptr == _data)
			return -ENOMEM;
	}

	/* If write size does not match, that is an error */
	if (_meta->o_size != buflen)
		return -EIO;

	/* Perform an atomic copy. */
	irqstate_t flags = irqsave();
	memcpy(_data, buffer, _meta->o_size);
	irqrestore(flags);

	/* update the timestamp and generation count */
	_last_update = hrt_absolute_time();
	_generation++;

	/* notify any poll waiters */
	poll_notify(POLLIN);

	return _meta->o_size;
}

int
ORBDevNode::ioctl(struct file *filp, int cmd, unsigned long arg)
{
	SubscriberData *sd = filp_to_sd(filp);

	switch (cmd) {
	case ORBIOCLASTUPDATE:
		*(hrt_abstime *)arg = _last_update;
		return OK;

	case ORBIOCUPDATED:
		*(bool *)arg = appears_updated(sd);
		return OK;

	case ORBIOCSETINTERVAL:
		sd->update_interval = arg;
		return OK;

	case ORBIOCGADVERTISER:
		*(uintptr_t *)arg = (uintptr_t)this;
		return OK;

	default:
		/* give it to the superclass */
		return CDev::ioctl(filp, cmd, arg);
	}
}

ssize_t
ORBDevNode::publish(const orb_metadata *meta, orb_advert_t handle, const void *data)
{
	ORBDevNode *devnode = (ORBDevNode *)handle;
	int ret;

	/* this is a bit risky, since we are trusting the handle in order to deref it */
	if (devnode->_meta != meta) {
		errno = EINVAL;
		return ERROR;
	}

	/* call the devnode write method with no file pointer */
	ret = devnode->write(nullptr, (const char *)data, meta->o_size);

	if (ret < 0)
		return ERROR;

	if (ret != (int)meta->o_size) {
		errno = EIO;
		return ERROR;
	}

	return OK;
}

pollevent_t
ORBDevNode::poll_state(struct file *filp)
{
	SubscriberData *sd = filp_to_sd(filp);

	/*
	 * If the topic appears updated to the subscriber, say so.
	 */
	if (appears_updated(sd))
		return POLLIN;

	return 0;
}

void
ORBDevNode::poll_notify_one(struct pollfd *fds, pollevent_t events)
{
	SubscriberData *sd = filp_to_sd((struct file *)fds->priv);

	/*
	 * If the topic looks updated to the subscriber, go ahead and notify them.
	 */
	if (appears_updated(sd))
		CDev::poll_notify_one(fds, events);
}

bool
ORBDevNode::appears_updated(SubscriberData *sd)
{
	/* assume it doesn't look updated */
	bool ret = false;

	/* avoid racing between interrupt and non-interrupt context calls */
	irqstate_t state = irqsave();

	/* check if this topic has been published yet, if not bail out */
	if (_data == nullptr) {
		ret = false;
		goto out;
	}

	/*
	 * If the subscriber's generation count matches the update generation
	 * count, there has been no update from their perspective; if they
	 * don't match then we might have a visible update.
	 */
	while (sd->generation != _generation) {

		/*
		 * Handle non-rate-limited subscribers.
		 */
		if (sd->update_interval == 0) {
			ret = true;
			break;
		}

		/*
		 * If we have previously told the subscriber that there is data,
		 * and they have not yet collected it, continue to tell them
		 * that there has been an update.  This mimics the non-rate-limited
		 * behaviour where checking / polling continues to report an update
		 * until the topic is read.
		 */
		if (sd->update_reported) {
			ret = true;
			break;
		}

		/*
		 * If the interval timer is still running, the topic should not
		 * appear updated, even though at this point we know that it has.
		 * We have previously been through here, so the subscriber
		 * must have collected the update we reported, otherwise
		 * update_reported would still be true.
		 */
		if (!hrt_called(&sd->update_call))
			break;

		/*
		 * Make sure that we don't consider the topic to be updated again
		 * until the interval has passed once more by restarting the interval
		 * timer and thereby re-scheduling a poll notification at that time.
		 */
		hrt_call_after(&sd->update_call,
			       sd->update_interval,
			       &ORBDevNode::update_deferred_trampoline,
			       (void *)this);

		/*
		 * Remember that we have told the subscriber that there is data.
		 */
		sd->update_reported = true;
		ret = true;

		break;
	}

out:
	irqrestore(state);

	/* consider it updated */
	return ret;
}

void
ORBDevNode::update_deferred()
{
	/*
	 * Instigate a poll notification; any subscribers whose intervals have
	 * expired will be woken.
	 */
	poll_notify(POLLIN);
}

void
ORBDevNode::update_deferred_trampoline(void *arg)
{
	ORBDevNode *node = (ORBDevNode *)arg;

	node->update_deferred();
}

/**
 * Master control device for ObjDev.
 *
 * Used primarily to create new objects via the ORBIOCCREATE
 * ioctl.
 */
class ORBDevMaster : public device::CDev
{
public:
	ORBDevMaster(Flavor f);
	~ORBDevMaster();

	virtual int		ioctl(struct file *filp, int cmd, unsigned long arg);
private:
	Flavor			_flavor;
};

ORBDevMaster::ORBDevMaster(Flavor f) :
	CDev((f == PUBSUB) ? "obj_master" : "param_master",
	     (f == PUBSUB) ? TOPIC_MASTER_DEVICE_PATH : PARAM_MASTER_DEVICE_PATH),
	_flavor(f)
{
	// enable debug() calls
	_debug_enabled = true;

}

ORBDevMaster::~ORBDevMaster()
{
}

int
ORBDevMaster::ioctl(struct file *filp, int cmd, unsigned long arg)
{
	int ret;

	switch (cmd) {
	case ORBIOCADVERTISE: {
			const struct orb_metadata *meta = (const struct orb_metadata *)arg;
			const char *objname;
			char nodepath[orb_maxpath];
			ORBDevNode *node;

			/* construct a path to the node - this also checks the node name */
			ret = node_mkpath(nodepath, _flavor, meta);

			if (ret != OK)
				return ret;

			/* driver wants a permanent copy of the node name, so make one here */
			objname = strdup(meta->o_name);

			if (objname == nullptr)
				return -ENOMEM;

			/* construct the new node */
			node = new ORBDevNode(meta, objname, nodepath);

			/* initialise the node - this may fail if e.g. a node with this name already exists */
			if (node != nullptr)
				ret = node->init();

			/* if we didn't get a device, that's bad */
			if (node == nullptr)
				return -ENOMEM;

			/* if init failed, discard the node and its name */
			if (ret != OK) {
				delete node;
				free((void *)objname);
			}

			return ret;
		}

	default:
		/* give it to the superclass */
		return CDev::ioctl(filp, cmd, arg);
	}
}


/**
 * Local functions in support of the shell command.
 */

namespace
{

ORBDevMaster	*g_dev;

struct orb_test {
	int val;
};

ORB_DEFINE(orb_test, struct orb_test);

int
test_fail(const char *fmt, ...)
{
	va_list ap;

	fprintf(stderr, "FAIL: ");
	va_start(ap, fmt);
	vfprintf(stderr, fmt, ap);
	va_end(ap);
	fprintf(stderr, "\n");
	fflush(stderr);
	return ERROR;
}

int
test_note(const char *fmt, ...)
{
	va_list ap;

	fprintf(stderr, "note: ");
	va_start(ap, fmt);
	vfprintf(stderr, fmt, ap);
	va_end(ap);
	fprintf(stderr, "\n");
	fflush(stderr);
	return OK;
}

ORB_DECLARE(sensor_combined);

int
test()
{
	struct orb_test t, u;
	int pfd, sfd;
	bool updated;

	t.val = 0;
	pfd = orb_advertise(ORB_ID(orb_test), &t);

	if (pfd < 0)
		return test_fail("advertise failed: %d", errno);

	test_note("publish handle 0x%08x", pfd);
	sfd = orb_subscribe(ORB_ID(orb_test));

	if (sfd < 0)
		return test_fail("subscribe failed: %d", errno);

	test_note("subscribe fd %d", sfd);
	u.val = 1;

	if (OK != orb_copy(ORB_ID(orb_test), sfd, &u))
		return test_fail("copy(1) failed: %d", errno);

	if (u.val != t.val)
		return test_fail("copy(1) mismatch: %d expected %d", u.val, t.val);

	if (OK != orb_check(sfd, &updated))
		return test_fail("check(1) failed");

	if (updated)
		return test_fail("spurious updated flag");

	t.val = 2;
	test_note("try publish");

	if (OK != orb_publish(ORB_ID(orb_test), pfd, &t))
		return test_fail("publish failed");

	if (OK != orb_check(sfd, &updated))
		return test_fail("check(2) failed");

	if (!updated)
		return test_fail("missing updated flag");

	if (OK != orb_copy(ORB_ID(orb_test), sfd, &u))
		return test_fail("copy(2) failed: %d", errno);

	if (u.val != t.val)
		return test_fail("copy(2) mismatch: %d expected %d", u.val, t.val);

	orb_unsubscribe(sfd);
	close(pfd);

#if 0
	/* this is a hacky test that exploits the sensors app to test rate-limiting */

	sfd = orb_subscribe(ORB_ID(sensor_combined));

	hrt_abstime start, end;
	unsigned count;

	start = hrt_absolute_time();
	count = 0;

	do {
		orb_check(sfd, &updated);

		if (updated) {
			orb_copy(ORB_ID(sensor_combined), sfd, nullptr);
			count++;
		}
	} while (count < 100);

	end = hrt_absolute_time();
	test_note("full-speed, 100 updates in %llu", end - start);

	orb_set_interval(sfd, 10);

	start = hrt_absolute_time();
	count = 0;

	do {
		orb_check(sfd, &updated);

		if (updated) {
			orb_copy(ORB_ID(sensor_combined), sfd, nullptr);
			count++;
		}
	} while (count < 100);

	end = hrt_absolute_time();
	test_note("100Hz, 100 updates in %llu", end - start);

	orb_unsubscribe(sfd);
#endif

	return test_note("PASS");
}

int
info()
{
	return OK;
}


} // namespace

/*
 * uORB server 'main'.
 */
extern "C" { __EXPORT int uorb_main(int argc, char *argv[]); }

int
uorb_main(int argc, char *argv[])
{
	/*
	 * Start/load the driver.
	 *
	 * XXX it would be nice to have a wrapper for this...
	 */
	if (!strcmp(argv[1], "start")) {

		if (g_dev != nullptr) {
			fprintf(stderr, "[uorb] already loaded\n");
			/* user wanted to start uorb, its already running, no error */
			return 0;
		}

		/* create the driver */
		g_dev = new ORBDevMaster(PUBSUB);

		if (g_dev == nullptr) {
			fprintf(stderr, "[uorb] driver alloc failed\n");
			return -ENOMEM;
		}

		if (OK != g_dev->init()) {
			fprintf(stderr, "[uorb] driver init failed\n");
			delete g_dev;
			g_dev = nullptr;
			return -EIO;
		}

		printf("[uorb] ready\n");
		return OK;
	}

	/*
	 * Test the driver/device.
	 */
	if (!strcmp(argv[1], "test"))
		return test();

	/*
	 * Print driver information.
	 */
	if (!strcmp(argv[1], "status"))
		return info();

	fprintf(stderr, "unrecognised command, try 'start', 'test' or 'status'\n");
	return -EINVAL;
}

/*
 * Library functions.
 */
namespace
{

void debug(const char *fmt, ...)
{
	va_list ap;

	va_start(ap, fmt);
	vfprintf(stderr, fmt, ap);
	va_end(ap);
	fprintf(stderr, "\n");
	fflush(stderr);
	usleep(100000);
}

/**
 * Advertise a node; don't consider it an error if the node has
 * already been advertised.
 *
 * @todo verify that the existing node is the same as the one
 *       we tried to advertise.
 */
int
node_advertise(const struct orb_metadata *meta)
{
	int fd = -1;
	int ret = ERROR;

	/* open the control device */
	fd = open(TOPIC_MASTER_DEVICE_PATH, 0);

	if (fd < 0)
		goto out;

	/* advertise the object */
	ret = ioctl(fd, ORBIOCADVERTISE, (unsigned long)(uintptr_t)meta);

	/* it's OK if it already exists */
	if ((OK != ret) && (EEXIST == errno))
		ret = OK;

out:

	if (fd >= 0)
		close(fd);

	return ret;
}

/**
 * Common implementation for orb_advertise and orb_subscribe.
 *
 * Handles creation of the object and the initial publication for
 * advertisers.
 */
int
node_open(Flavor f, const struct orb_metadata *meta, const void *data, bool advertiser)
{
	char path[orb_maxpath];
	int fd, ret;

	/*
	 * If meta is null, the object was not defined, i.e. it is not
	 * known to the system.  We can't advertise/subscribe such a thing.
	 */
	if (nullptr == meta) {
		errno = ENOENT;
		return ERROR;
	}

	/*
	 * Advertiser must publish an initial value.
	 */
	if (advertiser && (data == nullptr)) {
		errno = EINVAL;
		return ERROR;
	}

	/*
	 * Generate the path to the node and try to open it.
	 */
	ret = node_mkpath(path, f, meta);

	if (ret != OK) {
		errno = -ret;
		return ERROR;
	}

	/* open the path as either the advertiser or the subscriber */
	fd = open(path, (advertiser) ? O_WRONLY : O_RDONLY);

	/* we may need to advertise the node... */
	if (fd < 0) {

		/* try to create the node */
		ret = node_advertise(meta);

		/* on success, try the open again */
		if (ret == OK)
			fd = open(path, (advertiser) ? O_WRONLY : O_RDONLY);
	}

	if (fd < 0) {
		errno = EIO;
		return ERROR;
	}

	/* everything has been OK, we can return the handle now */
	return fd;
}

} // namespace

orb_advert_t
orb_advertise(const struct orb_metadata *meta, const void *data)
{
	int result, fd;
	orb_advert_t advertiser;

	/* open the node as an advertiser */
	fd = node_open(PUBSUB, meta, data, true);
	if (fd == ERROR)
		return ERROR;

	/* get the advertiser handle and close the node */
	result = ioctl(fd, ORBIOCGADVERTISER, (unsigned long)&advertiser);
	close(fd);
	if (result == ERROR)
		return ERROR;

	/* the advertiser must perform an initial publish to initialise the object */
	result= orb_publish(meta, advertiser, data);
	if (result == ERROR)
		return ERROR;

	return advertiser;
}

int
orb_subscribe(const struct orb_metadata *meta)
{
	return node_open(PUBSUB, meta, nullptr, false);
}

int
orb_unsubscribe(int handle)
{
	return close(handle);
}

int
orb_publish(const struct orb_metadata *meta, orb_advert_t handle, const void *data)
{
	return ORBDevNode::publish(meta, handle, data);
}

int
orb_copy(const struct orb_metadata *meta, int handle, void *buffer)
{
	int ret;

	ret = read(handle, buffer, meta->o_size);

	if (ret < 0)
		return ERROR;

	if (ret != (int)meta->o_size) {
		errno = EIO;
		return ERROR;
	}

	return OK;
}

int
orb_check(int handle, bool *updated)
{
	return ioctl(handle, ORBIOCUPDATED, (unsigned long)(uintptr_t)updated);
}

int
orb_stat(int handle, uint64_t *time)
{
	return ioctl(handle, ORBIOCLASTUPDATE, (unsigned long)(uintptr_t)time);
}

int
orb_set_interval(int handle, unsigned interval)
{
	return ioctl(handle, ORBIOCSETINTERVAL, interval * 1000);
}

