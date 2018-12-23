CXX     = g++
CFLAGS  = -g -O3 -Wall -std=c++0x -pthread
LIBS    = -lpthread -lasound -lwiringPi
LDFLAGS = -g

OBJECTS = Biquad.o CalDMR.o CalDStarRX.o CalDStarTX.o CalNXDN.o CalP25.o CWIdTX.o DMRDMORX.o DMRDMOTX.o \
	  DMRSlotType.o DStarRX.o DStarTX.o FIR.o FIRInterpolator.o IO.o IOUDRC.o MMDVM.o NXDNRX.o \
	  NXDNTX.o P25RX.o P25TX.o POCSAGTX.o SampleRB.o SerialPort.o SerialRB.o \
	  SoundCardReaderWriter.o Thread.o Utils.o YSFRX.o YSFTX.o

all:	MMDVM

MMDVM:	$(OBJECTS)
	$(CXX) $(OBJECTS) $(LDFLAGS) $(LIBS) -o MMDVM

-include $(OBJECTS:.o=.d)

%.o: %.cpp
	$(CXX) $(CFLAGS) -c -o $@ $<
	$(CXX) -MM $(CFLAGS) $< > $*.d

clean:
	$(RM) MMDVM *.o *.d *.bak *~

