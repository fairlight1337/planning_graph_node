#!/bin/bash

FILE=$1
EXEC_DIR=$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )


if [[ -z "${FILE}" ]]; then
    FILE="${EXEC_DIR}/../data/cram_log.owl"
fi


TARGET_DIR=${EXEC_DIR}/../data_created
mkdir -p ${TARGET_DIR}


cd ${EXEC_DIR}
for mode in "experiences" "condensed" "deduced"; do
    echo "-- Entering mode '${mode}'"
    
    ./protoexp/protoexp.py ${FILE} ${mode} > ${TARGET_DIR}/temp_${mode}.dot
    dot -Tpdf ${TARGET_DIR}/temp_${mode}.dot > ${TARGET_DIR}/${mode}.pdf
    rm ${TARGET_DIR}/temp_${mode}.dot
    
    echo "Created '${mode}.pdf'"
    
    echo "-- Completed mode '${mode}'"
done
cd - > /dev/null

mv ${EXEC_DIR}/deduced_experiences.json ${TARGET_DIR}
