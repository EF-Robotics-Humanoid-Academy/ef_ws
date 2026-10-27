import os
import tempfile

# CycloneDDS encodes config XML as ASCII, so the log path must be ASCII-only.
# Fall back to a relative path if the system temp dir contains non-ASCII chars.
_cdds_log_path = os.path.join(tempfile.gettempdir(), "cdds.LOG").replace("\\", "/")
try:
    _cdds_log_path.encode("ascii")
except UnicodeEncodeError:
    _cdds_log_path = "cdds.LOG"

ChannelConfigHasInterface = '''<?xml version="1.0" encoding="UTF-8" ?>
    <CycloneDDS>
        <Domain Id="any">
            <General>
                <Interfaces>
                    <NetworkInterface name="$__IF_NAME__$" priority="default" multicast="default"/>
                </Interfaces>
            </General>
            <Tracing>
                <Verbosity>config</Verbosity>
            <OutputFile>''' + _cdds_log_path + '''</OutputFile>
        </Tracing>
        </Domain>
    </CycloneDDS>'''

ChannelConfigAutoDetermine = '''<?xml version="1.0" encoding="UTF-8" ?>
    <CycloneDDS>
        <Domain Id="any">
            <General>
                <Interfaces>
                    <NetworkInterface autodetermine=\"true\" priority=\"default\" multicast=\"default\" />
                </Interfaces>
            </General>
        </Domain>
    </CycloneDDS>'''
