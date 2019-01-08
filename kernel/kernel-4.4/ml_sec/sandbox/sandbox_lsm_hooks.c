/*
 * Copyright (c) 2017, Magic Leap, Inc. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

/*
 * This file contains the implementations of the syscall inspection
 * functions, as well as their function table.
 */

#include <linux/ml_sec/sandbox.h>
#include <linux/init.h>
#include <linux/lsm_hooks.h>
#include <linux/inetdevice.h>
#include <net/af_unix.h>
#include <net/ipv6.h>

struct portrange {
	unsigned short min_port;
	unsigned short max_port;
};

/* Port ranges on localhost that jailed debuggable apps can connect to (inclusive) */
static const struct portrange ALLOWED_PORTRANGES[] = {  {57000, 57100},
						 	{41898, 41899}, };

static bool find_ipv4_address(struct sock *sk, __be32 addr)
{
	struct net *net = sock_net(sk);
	struct net_device *dev;
	struct in_device *in_dev;
	bool retval = true;

	rcu_read_lock();
	for_each_netdev_rcu(net, dev) {
		if (dev->reg_state >= NETREG_UNREGISTERING)
			continue;

		in_dev = __in_dev_get_rcu(dev);
		if (!in_dev)
			continue;

		for_ifa(in_dev) {
			if (addr == ifa->ifa_address)
				goto out;
		} endfor_ifa(in_dev);
	}

	retval = false;
out:
	rcu_read_unlock();
	return retval;
}

static bool find_ipv6_address(struct sock *sk, const struct in6_addr *addr)
{
	struct net *net = sock_net(sk);
	struct net_device *dev;
	struct inet6_dev *in6_dev;
	struct inet6_ifaddr *ifa;
	bool retval = true;

	rcu_read_lock();
	for_each_netdev_rcu(net, dev) {
		if (dev->reg_state >= NETREG_UNREGISTERING)
			continue;

		in6_dev = rcu_dereference(dev->ip6_ptr);
		if (!in6_dev)
			continue;

		list_for_each_entry(ifa, &in6_dev->addr_list, if_list) {
			if (ipv6_addr_equal(addr, &ifa->addr))
				goto out;
		}
	}

	retval = false;
out:
	rcu_read_unlock();
	return retval;
}

static inline bool is_local_ipv4_addr(struct sock *sk, __be32 addr)
{
	return IN_LOOPBACK(ntohl(addr)) ||
	       addr == INADDR_ANY       ||
	       find_ipv4_address(sk, addr);
}

static bool is_local_ipv6_addr(struct sock *sk, const struct in6_addr *addr)
{
	if (ipv6_addr_v4mapped(addr))
		return is_local_ipv4_addr(sk, addr->s6_addr32[3]);

	return ipv6_addr_loopback(addr) ||
	       ipv6_addr_any(addr)      ||
	       find_ipv6_address(sk, addr);
}

static int verify_sockaddr(struct sock *sk, struct sockaddr *address,
			   int addrlen, const char *op_name)
{
	unsigned short port;

	if (sk->sk_family == PF_INET) {
		struct sockaddr_in *addr4 = (struct sockaddr_in *)address;

		if (addrlen < sizeof(struct sockaddr_in))
			return -EINVAL;

		if (!is_local_ipv4_addr(sk, addr4->sin_addr.s_addr))
			return 0;

		port = ntohs(addr4->sin_port);
	} else if (sk->sk_family == PF_INET6) {
		struct sockaddr_in6 *addr6 = (struct sockaddr_in6 *)address;

		if (addrlen < SIN6_LEN_RFC2133)
			return -EINVAL;

		if (!is_local_ipv6_addr(sk, &addr6->sin6_addr))
			return 0;

		port = ntohs(addr6->sin6_port);
	} else
		return 0;

	if (current->is_debuggable) {
		size_t i;

		for (i = 0; i < ARRAY_SIZE(ALLOWED_PORTRANGES); i++) {
			if (ALLOWED_PORTRANGES[i].min_port <= port && port <= ALLOWED_PORTRANGES[i].max_port)
				return 0;
		}

	}



	sbox_audit_jail_localhost(op_name, port);
	if (!sbox_is_permissive())
		return -EPERM;

	return 0;
}

static int sbox_socket_connect(struct socket *sock, struct sockaddr *address,
			       int addrlen)
{
	if (current->sandbox_profile->allow_local_connections)
		return 0;

	return verify_sockaddr(sock->sk, address, addrlen, "connect");
}

static int sbox_socket_sendmsg(struct socket *sock, struct msghdr *msg,
			       int size)
{
	struct sockaddr *address;
	int addrlen;

	if (current->sandbox_profile->allow_local_connections)
		return 0;

	address = msg->msg_name;
	addrlen = msg->msg_namelen;
	if (!address || !addrlen)
		return 0;

	return verify_sockaddr(sock->sk, address, addrlen, "sendto");
}

static struct security_hook_list sbox_hooks[] = {
	LSM_HOOK_INIT(socket_connect, sbox_socket_connect),
	LSM_HOOK_INIT(socket_sendmsg, sbox_socket_sendmsg),
};

static __init int sbox_init(void)
{
	pr_info("sandbox:  Initializing.\n");
	security_add_hooks(sbox_hooks, ARRAY_SIZE(sbox_hooks));
	return 0;
}

/* Sandbox requires early initialization for applying security hooks. */
security_initcall(sbox_init);
