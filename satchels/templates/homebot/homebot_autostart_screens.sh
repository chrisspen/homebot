#!/bin/bash
LOG_FILE={{log_dir}}/{{autostart_script}}.log

echo "" >> ${LOG_FILE}
echo "" >> ${LOG_FILE}
echo "" >> ${LOG_FILE}
echo "" >> ${LOG_FILE}
echo "#############################################" >> ${LOG_FILE}
echo "Running {{autostart_script}}" >> ${LOG_FILE}
echo $(date) >> ${LOG_FILE}
echo "#############################################" >> ${LOG_FILE}
echo "" >> ${LOG_FILE}
echo "Logs:" >> ${LOG_FILE}

set -e
set -v

{

screen -d -m bash {{script_dir}}/{{start_script}}.sh

} &>> ${LOG_FILE}
