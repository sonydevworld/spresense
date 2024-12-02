
#include <nuttx/config.h>
#include <stdio.h>

#include <audiolite/audiolite.h>
#include "alusr_fftworker.h"

#define MAX_DISP_LEN  (50)

class fft_disp : public audiolite_component
{
  public:
    char gray_to_ascii(unsigned char d)
    {
      if (d < 0x10) return ' ';
      if (d < 0x20) return '.';
      if (d < 0x30) return ',';
      if (d < 0x40) return '_';
      if (d < 0x50) return '-';
      if (d < 0x60) return '~';
      if (d < 0x70) return '+';
      if (d < 0x80) return 'i';
      if (d < 0x90) return 'b';
      if (d < 0xa0) return 'p';
      if (d < 0xb0) return 'A';
      if (d < 0xc0) return 'H';
      if (d < 0xd0) return 'V';
      if (d < 0xe0) return 'O';
      return 'Z';
    };

    void on_data()
    {
      audiolite_memapbuf *mem = (audiolite_memapbuf *)pop_data();

      if (mem)
        {
          uint8_t *data = (uint8_t *)mem->get_data();

          for (int i = 0; i < MAX_DISP_LEN; i++)
            {
              printf("%c", gray_to_ascii(*data++));
            }

          printf("\n");
          mem->release();
        }
    };
};

extern "C" int main(int argc, FAR char *argv[])
{
  audiolite_simplelistener lsn;
  audiolite_mempoolapbuf *membool = new audiolite_mempoolapbuf;
  audiolite_inputcomp    *aindev  = new audiolite_inputcomp;
  alusr_fftworker        *fft     = new alusr_fftworker;
  fft_disp               *disp    = new fft_disp;

  audiolite_set_evtlistener(&lsn);
  audiolite_set_systemparam(48000, 16, 2);

  membool->create_instance(FFT_TAPS * 2 * 2, 16);
  aindev->set_mempool(membool);

  aindev->bind(fft)->bind(disp);
  aindev->start();

  for(int i = 0; i < 15; i++) sleep(1);

  printf("Stop audio\n");
  aindev->stop();

  aindev->unbindall();
  audiolite_eventdestroy();

  delete disp;
  delete fft;
  delete aindev;
  delete membool;

  return 0;
}
