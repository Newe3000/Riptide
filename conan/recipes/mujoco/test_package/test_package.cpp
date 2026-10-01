// Proves the packaged MuJoCo actually steps physics: build a tiny model, load
// it, and advance one step. Exercises the real libmujoco at runtime.
#include <mujoco/mujoco.h>

#include <cstdio>

int main()
{
  const char * xml =
    "<mujoco><worldbody>"
    "<body><freejoint/><geom type=\"box\" size=\".1 .1 .1\"/></body>"
    "</worldbody></mujoco>";
  const char * path = "/tmp/_conan_mujoco_test.xml";
  std::FILE * f = std::fopen(path, "w");
  if (!f) { std::printf("cannot write %s\n", path); return 1; }
  std::fputs(xml, f);
  std::fclose(f);

  char err[1024] = "";
  mjModel * m = mj_loadXML(path, nullptr, err, sizeof(err));
  if (!m) { std::printf("mj_loadXML failed: %s\n", err); return 1; }
  mjData * d = mj_makeData(m);
  mj_step(m, d);
  std::printf("mujoco version %d ok: nq=%d nv=%d stepped to t=%.4f\n",
              mj_version(), (int)m->nq, (int)m->nv, d->time);
  mj_deleteData(d);
  mj_deleteModel(m);
  return 0;
}
