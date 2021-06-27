# invoke SourceDir generated makefile for project.prm3
project.prm3: .libraries,project.prm3
.libraries,project.prm3: package/cfg/project_prm3.xdl
	$(MAKE) -f D:\Workspace\Projects\cc2650_demo\button/src/makefile.libs

clean::
	$(MAKE) -f D:\Workspace\Projects\cc2650_demo\button/src/makefile.libs clean

