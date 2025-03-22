#!/bin/bash

# Directories
PROTO_DIR="main/protobuf"
OUTPUT_DIR="main/protobuf"
NANOPB_GENERATOR="components/nanopb/generator/nanopb_generator"

# Ensure the output directory exists
mkdir -p "${OUTPUT_DIR}"

# Find all .proto files and generate .pb.c and .pb.h files
for PROTO_FILE in ${PROTO_DIR}/*.proto; do
    echo "Generating files for ${PROTO_FILE}..."
    ${NANOPB_GENERATOR} --proto-path=${PROTO_DIR} --output-dir=${OUTPUT_DIR} "${PROTO_FILE}"
done

echo "All .proto files processed! Files are in ${OUTPUT_DIR}"
