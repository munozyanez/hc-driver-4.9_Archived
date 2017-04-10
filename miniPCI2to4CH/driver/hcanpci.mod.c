#include <linux/module.h>
#include <linux/vermagic.h>
#include <linux/compiler.h>

MODULE_INFO(vermagic, VERMAGIC_STRING);

__visible struct module __this_module
__attribute__((section(".gnu.linkonce.this_module"))) = {
	.name = KBUILD_MODNAME,
	.init = init_module,
#ifdef CONFIG_MODULE_UNLOAD
	.exit = cleanup_module,
#endif
	.arch = MODULE_ARCH_INIT,
};

static const struct modversion_info ____versions[]
__used
__attribute__((section("__versions"))) = {
	{ 0x452cc29f, __VMLINUX_SYMBOL_STR(module_layout) },
	{ 0xa496bf6f, __VMLINUX_SYMBOL_STR(param_ops_int) },
	{ 0x3389ffeb, __VMLINUX_SYMBOL_STR(pci_unregister_driver) },
	{ 0x7485e15e, __VMLINUX_SYMBOL_STR(unregister_chrdev_region) },
	{ 0x69700ca, __VMLINUX_SYMBOL_STR(__pci_register_driver) },
	{ 0xe0c88ff5, __VMLINUX_SYMBOL_STR(proc_mkdir) },
	{ 0x29537c9e, __VMLINUX_SYMBOL_STR(alloc_chrdev_region) },
	{ 0xd8e484f0, __VMLINUX_SYMBOL_STR(register_chrdev_region) },
	{ 0xf20dabd8, __VMLINUX_SYMBOL_STR(free_irq) },
	{ 0xc0a027a1, __VMLINUX_SYMBOL_STR(cdev_del) },
	{ 0x6d2b75a4, __VMLINUX_SYMBOL_STR(cdev_add) },
	{ 0xa72bfb2e, __VMLINUX_SYMBOL_STR(cdev_init) },
	{ 0xc439bb8a, __VMLINUX_SYMBOL_STR(pci_disable_device) },
	{ 0x8806d005, __VMLINUX_SYMBOL_STR(pci_release_regions) },
	{ 0xedc03953, __VMLINUX_SYMBOL_STR(iounmap) },
	{ 0x1ed31e60, __VMLINUX_SYMBOL_STR(remove_proc_entry) },
	{ 0xd6b8e852, __VMLINUX_SYMBOL_STR(request_threaded_irq) },
	{ 0x524dc18, __VMLINUX_SYMBOL_STR(proc_create_data) },
	{ 0x20c55ae0, __VMLINUX_SYMBOL_STR(sscanf) },
	{ 0xa3e09964, __VMLINUX_SYMBOL_STR(pci_bus_read_config_byte) },
	{ 0x42c8de35, __VMLINUX_SYMBOL_STR(ioremap_nocache) },
	{ 0x9e88526, __VMLINUX_SYMBOL_STR(__init_waitqueue_head) },
	{ 0xcd752e3, __VMLINUX_SYMBOL_STR(pci_request_regions) },
	{ 0xe26e916, __VMLINUX_SYMBOL_STR(pci_enable_device) },
	{ 0x727c4f3, __VMLINUX_SYMBOL_STR(iowrite8) },
	{ 0x88db9f48, __VMLINUX_SYMBOL_STR(__check_object_size) },
	{ 0xd2b09ce5, __VMLINUX_SYMBOL_STR(__kmalloc) },
	{ 0xb5419b40, __VMLINUX_SYMBOL_STR(_copy_from_user) },
	{ 0x37a0cba, __VMLINUX_SYMBOL_STR(kfree) },
	{ 0xc943d52e, __VMLINUX_SYMBOL_STR(kmem_cache_alloc_trace) },
	{ 0xdcb8916d, __VMLINUX_SYMBOL_STR(kmalloc_caches) },
	{ 0xd62c833f, __VMLINUX_SYMBOL_STR(schedule_timeout) },
	{ 0x78e739aa, __VMLINUX_SYMBOL_STR(up) },
	{ 0xf9a482f9, __VMLINUX_SYMBOL_STR(msleep) },
	{ 0x436c2179, __VMLINUX_SYMBOL_STR(iowrite32) },
	{ 0xe0afe59e, __VMLINUX_SYMBOL_STR(down_interruptible) },
	{ 0xdb7305a1, __VMLINUX_SYMBOL_STR(__stack_chk_fail) },
	{ 0xf08242c2, __VMLINUX_SYMBOL_STR(finish_wait) },
	{ 0x2207a57f, __VMLINUX_SYMBOL_STR(prepare_to_wait_event) },
	{ 0x1000e51, __VMLINUX_SYMBOL_STR(schedule) },
	{ 0x622598b1, __VMLINUX_SYMBOL_STR(init_wait_entry) },
	{ 0xc671e369, __VMLINUX_SYMBOL_STR(_copy_to_user) },
	{ 0xe484e35f, __VMLINUX_SYMBOL_STR(ioread32) },
	{ 0xa1c76e0a, __VMLINUX_SYMBOL_STR(_cond_resched) },
	{ 0xf10de535, __VMLINUX_SYMBOL_STR(ioread8) },
	{ 0x91715312, __VMLINUX_SYMBOL_STR(sprintf) },
	{ 0x2f0b2ec3, __VMLINUX_SYMBOL_STR(PDE_DATA) },
	{ 0xa6bbd805, __VMLINUX_SYMBOL_STR(__wake_up) },
	{ 0x8c183cbe, __VMLINUX_SYMBOL_STR(iowrite16) },
	{ 0x27e1a049, __VMLINUX_SYMBOL_STR(printk) },
	{ 0xc5534d64, __VMLINUX_SYMBOL_STR(ioread16) },
	{ 0xbdfb6dbb, __VMLINUX_SYMBOL_STR(__fentry__) },
};

static const char __module_depends[]
__used
__attribute__((section(".modinfo"))) =
"depends=";

MODULE_ALIAS("pci:v000010B5d00009030sv000010B5sd00003267bc*sc*i*");
MODULE_ALIAS("pci:v000010B5d00009030sv000010B5sd00003325bc*sc*i*");
