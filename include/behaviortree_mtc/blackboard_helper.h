#pragma once

#include <memory>

#include <behaviortree_cpp/tree_node.h>

#include <behaviortree_mtc/shared_to_unique.h>

namespace BT {
namespace MTC {

/** Converts a shared pointer to a unique pointer from the blackboard (threadsafe)
 * The blackboard content is assigned to nullptr.
 * Throws if the shared pointer count is greater the one.
 * Throws if the shared pointer has not the required deleter `fake_deleter` (see unique_to_shared.h).
 * Throws if the cast to the blackboard entry is the wrong type or Any is empty
 *
 * @param any_locked_ptr pointer to Any, protected with a locked mutex as long as the object is in scope
 * @return               a unique pointer converted from the shared pointer
 */
template <typename T>
std::unique_ptr<T> convertSharedToUniqueLocked(BT::AnyPtrLocked& any_locked_ptr)
{
  std::unique_ptr<T> unique_t{ nullptr };

  if(auto* t_ptr = any_locked_ptr->castPtr<std::shared_ptr<T>>())
  {
    auto& t = *t_ptr;

    // actual conversion
    unique_t = sharedToUnique(std::move(t));

    // Is this really necessary ?
    // set blackboard value to nullptr
    any_locked_ptr.assign(nullptr);

    return unique_t;
  }
  throw std::runtime_error("tried to cast a blackboard entry to the wrong type "
                           "or Any is empty");
}

/** Converts a shared pointer to a unique pointer from a blackboard TreeNode (threadsafe)
 * The blackboard content is assigned to nullptr.
 * Throws if the shared pointer count is greater the one.
 * Throws if the shared pointer has not the required deleter `fake_deleter` (see unique_to_shared.h).
 * Throws if the cast to the blackboard entry is the wrong type or Any is empty
 * Throws if the blackboard entry does not exists or the content of the port was a static string
 *
 * @param tree_node this tree node.
 * @param key       the blackboard object key.
 * @return          a unique pointer converted from the shared pointer
 */
template <typename T>
std::unique_ptr<T> convertSharedToUniqueLocked(BT::TreeNode& tree_node, const std::string& key)
{
  std::unique_ptr<T> unique_t{ nullptr };

  if(auto any_locked_ptr = tree_node.getLockedPortContent(key))
  {
    return convertSharedToUniqueLocked<T>(any_locked_ptr);
  }

  throw std::runtime_error("the blackboard entry doesn't exist or the content "
                           "of the port was a static string");
}

/** Converts a shared pointer to a unique pointer from a blackboard (threadsafe)
 * The blackboard content is assigned to nullptr.
 * Throws if the shared pointer count is greater the one.
 * Throws if the shared pointer has not the required deleter `fake_deleter` (see unique_to_shared.h).
 * Throws if the cast to the blackboard entry is the wrong type or Any is empty
 * Throws if the blackboard entry does not exists or the content of the port was a static string
 *
 * @param blackboard the blackboard.
 * @param key        the blackboard object key.
 * @return           a unique pointer converted from the shared pointer
 */
template <typename T>
std::unique_ptr<T> convertSharedToUniqueLocked(BT::Blackboard& blackboard, const std::string& key)
{
  std::unique_ptr<T> unique_t{ nullptr };

  if(auto any_locked_ptr = blackboard.getAnyLocked(key))
  {
    return convertSharedToUniqueLocked<T>(any_locked_ptr);
  }

  throw std::runtime_error("the blackboard entry doesn't exist or the content "
                           "of the port was a static string");
}

}  // namespace MTC
}  // namespace BT
