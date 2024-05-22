#pragma once

#include <memory>
#include <stdexcept>

namespace bt_mtc
{
namespace dirty
{
// if disabled, does not actually delete pointer
// enabled by default (same as std::default_delete)
// https://qr.ae/psaWhg
struct fake_deleter
{
  void disable()
  {
    really_delete_ = false;
  }

  template <class T>
  void operator()(T* p)
  {
    if(really_delete_)
      delete p;
  }

private:
  bool really_delete_ = true;
};
}  // namespace dirty

template <typename T>
std::unique_ptr<T> sharedToUnique(std::shared_ptr<T>& shp)
{
  std::unique_ptr<T> up{ nullptr };

  if(shp.use_count() != 1)
    throw std::runtime_error("shared ptr has other owners, cannot convert to unique.");

  auto shp_delp = std::get_deleter<dirty::fake_deleter>(shp);

  if(!shp_delp)
    throw std::runtime_error("shared ptr is missing the deleter: 'dirty::fake_deleter', "
                             "cannot convert to "
                             "unique.");

  shp_delp->disable();  // disable deletion in shp

  up.reset(shp.get());  // obtain pointer

  return std::move(up);
}
}  // namespace bt_mtc
