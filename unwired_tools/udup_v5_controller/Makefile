deploy_ud: update_version
	scp *.lua root@192.168.20.187:/root/udup_v5_controller && \
	osascript -e 'display notification with title "Script updated" subtitle "192.168.20.187 (Unwired Devices office)"'

deploy_home: update_version
	scp *.lua root@192.168.88.110:/root/udup_v5_controller && \
	osascript -e 'display notification with title "Script updated" subtitle "192.168.88.110 (Home)"'

deploy_lan: update_version
	scp *.lua root@192.168.97.1:/root/udup_v5_controller && \
	osascript -e 'display notification with title "Script updated" subtitle "192.168.97.1 (LAN)"'

deploy_auto:
	watchmedo shell-command --drop --command='make'

update_version:
	echo "version = {}\nversion.git = '`git describe --dirty --always --tags`'\nreturn version" > version.lua



