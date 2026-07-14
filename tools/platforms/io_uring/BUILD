package(default_visibility = ["//visibility:public"])

# Whether a platform can run //aos/events:aio's io_uring backend.  Aio always
# builds both backends on Linux; this picks which one --aio_backend defaults
# to, compiled in (see AOS_AIO_DEFAULT_BACKEND in //aos/events:aio).
#
# io_uring needs a kernel >= 6.12 and there is deliberately no runtime
# fallback -- a loop asked for io_uring on a kernel that lacks it fails
# loudly rather than quietly degrading -- so the choice has to be made per
# deployment, and the platform is what knows.
#
# The default is :lacks_support so that a platform which never says (an
# out-of-tree one, say) gets the epoll backend, which runs anywhere.  Every
# platform in //tools/platforms states it explicitly regardless.
constraint_setting(
    name = "io_uring_support",
    default_constraint_value = ":lacks_support",
)

constraint_value(
    name = "has_support",
    constraint_setting = ":io_uring_support",
)

constraint_value(
    name = "lacks_support",
    constraint_setting = ":io_uring_support",
)
