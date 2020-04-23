PART_NAME=firmware

platform_check_image() {
	return 0;
}

platform_do_upgrade() {
	default_do_upgrade "$1"
}
