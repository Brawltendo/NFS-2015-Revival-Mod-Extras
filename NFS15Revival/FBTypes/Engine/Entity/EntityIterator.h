#pragma once

#include <Engine/Entity/EntityDeclarations.h>
#include <cstdint>

namespace fb
{

class EntityIterator
{
public:
	EntityIterableLink* getFirstIterableLink(const uintptr_t classInfo, uintptr_t gameWorld);
};

template <uintptr_t EntityType, bool IncludeNonIterable = false>
class TypedEntityIterator : EntityIterator
{
public:
	TypedEntityIterator(uintptr_t entityWorld);
	uintptr_t next(uint32_t entityOffset);
	/*template <class ET>
	ET* next(uint32_t entityOffset)
	{
		return reinterpret_cast<ET*>(next(0x40));
	}*/

	const EntityIterableLink* link() { return m_link; }

private:
	uintptr_t m_entityWorld;
	const EntityIterableLink* m_link;
	bool m_includeNonIterable;
};


EntityIterableLink* EntityIterator::getFirstIterableLink(const uintptr_t classInfo, uintptr_t gameWorld)
{
	EntityIterableLink*(__fastcall* native)(uintptr_t, uintptr_t) = reinterpret_cast<EntityIterableLink*(__fastcall*)(uintptr_t, uintptr_t)>(0x1437A2DE0);
	return native(classInfo, gameWorld);
}

template <uintptr_t EntityType, bool IncludeNonIterable>
TypedEntityIterator<EntityType, IncludeNonIterable>::TypedEntityIterator(uintptr_t entityWorld)
	: m_entityWorld(entityWorld)
	, m_link(getFirstIterableLink(EntityType, entityWorld))
	, m_includeNonIterable(IncludeNonIterable)
{
}

template <uintptr_t EntityType, bool IncludeNonIterable>
uintptr_t TypedEntityIterator<EntityType, IncludeNonIterable>::next(uint32_t entityOffset)
{
	// go up to entity from iterator
	uintptr_t entity = (uintptr_t)m_link - entityOffset;
	m_link = m_link->next;
	return entity;
}

}
