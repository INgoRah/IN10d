# called after build
Import("env", "projenv")
import subprocess

tagcmd = "git describe --tags --abbrev=0"
try:
	version = subprocess.check_output(tagcmd, shell=True).decode().strip()
except Exception:
	version = "1.0"
revcmd = "git log --pretty=format:%h -n 1"
try:
	commit = subprocess.check_output(revcmd, shell=True).decode().strip()
except Exception:
	commit = "deadbeef"
rev = '{}-{}'.format(version, commit)
print("Version " + rev)

env.AddPostAction(
    "$BUILD_DIR/${PROGNAME}.elf",
    env.VerboseAction(" ".join([
        "avr-objdump -d -S -j .text ",
        "$BUILD_DIR/${PROGNAME}.elf", ">", "$BUILD_DIR/${PROGNAME}.dis"
    ]), "Creating disasm ${PROGNAME}.dis")
)# Custom HEX from ELF
env.AddPostAction(
    "$BUILD_DIR/${PROGNAME}.hex",
    env.VerboseAction(" ".join([
        "cp ",
        "$BUILD_DIR/${PROGNAME}.hex", "./${PROGNAME}-" + rev + ".hex"
    ]), "Copying $BUILD_DIR/${PROGNAME}-" + rev + ".hex")
)
