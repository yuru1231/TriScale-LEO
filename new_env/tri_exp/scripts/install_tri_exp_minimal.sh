#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Usage:
  scripts/install_tri_exp_minimal.sh --ns3 /path/to/ns-3.43 [--tri-exp /path/to/tri_exp] [--apply]

Default mode is dry-run. It validates the minimal install manifest and reports
which files would be replaced or added. Use --apply only against a clean-room
copy of ns-3.43 after reviewing the dry-run output.
EOF
}

tri_exp="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ns3=""
apply=0

while [[ $# -gt 0 ]]; do
  case "$1" in
    --tri-exp)
      tri_exp="$2"
      shift 2
      ;;
    --ns3)
      ns3="$2"
      shift 2
      ;;
    --apply)
      apply=1
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "ERROR: unknown argument: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

if [[ -z "$ns3" ]]; then
  echo "ERROR: --ns3 is required" >&2
  usage >&2
  exit 2
fi

manifest="$tri_exp/INSTALL_MANIFEST.txt"

require_path() {
  local path="$1"
  local description="$2"
  if [[ ! -e "$path" ]]; then
    echo "ERROR: missing $description: $path" >&2
    exit 1
  fi
}

require_dir() {
  local path="$1"
  local description="$2"
  if [[ ! -d "$path" ]]; then
    echo "ERROR: missing $description: $path" >&2
    exit 1
  fi
}

copy_file() {
  local src="$1"
  local dst="$2"
  if [[ "$apply" -eq 1 ]]; then
    install -D "$src" "$dst"
  fi
}

section=""
replace_existing=0
add_new=0
scratch=0
same_existing=0
different_existing=0

require_dir "$tri_exp" "tri_exp directory"
require_dir "$ns3" "ns-3.43 directory"
require_path "$manifest" "install manifest"
require_path "$ns3/ns3" "ns3 launcher"
require_dir "$ns3/contrib/satellite" "satellite contrib module"
require_dir "$ns3/contrib/satellite/data" "satellite data directory"

if [[ "$apply" -eq 1 ]]; then
  echo "Mode: APPLY"
else
  echo "Mode: DRY-RUN"
fi

echo "tri_exp: $tri_exp"
echo "ns-3.43: $ns3"
echo

while IFS= read -r line || [[ -n "$line" ]]; do
  line="${line%%#*}"
  line="${line%"${line##*[![:space:]]}"}"
  line="${line#"${line%%[![:space:]]*}"}"

  [[ -z "$line" ]] && continue

  if [[ "$line" =~ ^\[.*\]$ ]]; then
    section="$line"
    continue
  fi

  case "$section" in
    "[replace_existing_satellite_files]")
      src="$tri_exp/$line"
      dst="$ns3/contrib/satellite/$line"
      require_path "$src" "source file"
      require_path "$dst" "target file to replace"
      replace_existing=$((replace_existing + 1))
      if cmp -s "$src" "$dst"; then
        same_existing=$((same_existing + 1))
        printf '[SAME]    %s\n' "$line"
      else
        different_existing=$((different_existing + 1))
        printf '[REPLACE] %s\n' "$line"
      fi
      copy_file "$src" "$dst"
      ;;
    "[add_new_satellite_files]")
      src="$tri_exp/$line"
      dst="$ns3/contrib/satellite/$line"
      require_path "$src" "source file"
      add_new=$((add_new + 1))
      if [[ -e "$dst" ]]; then
        if cmp -s "$src" "$dst"; then
          printf '[EXISTS-SAME] %s\n' "$line"
        else
          printf '[EXISTS-DIFF] %s\n' "$line"
        fi
      else
        printf '[ADD]     %s\n' "$line"
      fi
      copy_file "$src" "$dst"
      ;;
    "[replace_satellite_cmakelists]")
      src_rel="${line%% -> *}"
      dst_rel="${line##* -> }"
      src="$tri_exp/$src_rel"
      dst="$ns3/$dst_rel"
      require_path "$src" "source CMakeLists"
      require_path "$dst" "target CMakeLists"
      printf '[REPLACE] %s\n' "$dst_rel"
      copy_file "$src" "$dst"
      ;;
    "[install_scratch_files]")
      src="$tri_exp/$line"
      dst="$ns3/$line"
      require_path "$src" "scratch source file"
      scratch=$((scratch + 1))
      if [[ -e "$dst" ]]; then
        if cmp -s "$src" "$dst"; then
          printf '[SCRATCH-SAME] %s\n' "$line"
        else
          printf '[SCRATCH-REPLACE] %s\n' "$line"
        fi
      else
        printf '[SCRATCH-ADD] %s\n' "$line"
      fi
      copy_file "$src" "$dst"
      ;;
    *)
      echo "ERROR: manifest entry outside a known section: $line" >&2
      exit 1
      ;;
  esac
done < "$manifest"

echo
echo "Summary:"
echo "  replace-existing entries: $replace_existing"
echo "  existing entries same:    $same_existing"
echo "  existing entries differ:  $different_existing"
echo "  add-new entries:          $add_new"
echo "  scratch entries:          $scratch"

if [[ "$apply" -eq 1 ]]; then
  echo
  echo "Applied. Next recommended commands:"
  echo "  cd \"$ns3\""
  echo "  ./ns3 configure --enable-examples --enable-tests"
  echo "  ./ns3 build"
else
  echo
  echo "Dry-run only. No files were modified."
fi
