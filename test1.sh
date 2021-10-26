#! /bin/bash
cronjob="@reboot hey"
(crontab -u $USER -l; echo "$cronjob" ) | crontab -u $USER -
