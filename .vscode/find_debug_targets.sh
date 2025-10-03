#!/usr/bin/env bash

# Task or launch
type="$1"

# cc_binary or cc_test
target="$2"

if [[ "$type" == "launch" ]]; then
  bazel query 'kind('"$target"', ...)' --output label | sed 's\//\./\g' | sed 's\:\/\g'
elif [[ "$type" == "task" ]]; then
  bazel query 'kind('"$target"', ...)' --output label
else
  echo "Invalid type '$type', must be 'launch' or 'task'"
  exit -1
fi

